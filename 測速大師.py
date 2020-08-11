#!/usr/bin/env python
#-*- coding: utf-8 -*-
import numpy as np
import cv2
import os
import math
from collections import deque
import threading
import time

NUM_THREADS = 3

LoadFrameBarrier = threading.Barrier(NUM_THREADS)
WaitThreadBarrier = threading.Barrier(NUM_THREADS)
lock = threading.Lock()

car_num_lock = threading.Lock()

global frame,car_contour3,car_num,cap
car_num = 1

ExceedSpeed = 40

   

def LeftToRight_carTracking():
    
    fgbg = cv2.createBackgroundSubtractorMOG2()
    kernel = np.ones((21,21),np.uint8)  #影像處理時用到的mask大小
    #scale: 決定每公尺多少像素
    scale = 70
    allCarPos = []
    posList = []
    pictag = []
    global car_num
    #開始處理影格
    #while(cap.isOpened()):
    while(1):
        LoadFrameBarrier.wait()
          

        #獲知每秒多少影格frame per second
        fps = cap.get(cv2.CAP_PROP_FPS)
        
        """
        #影格播完了，就播放下一個，直到video_index到達影片檔案的數量，就停止
        if frame is None:

            video_index += 1
            if video_index >= len(videofiles):
                break
            cap = cv2.VideoCapture(videofiles[ video_index ])
            ret, frame = cap.read()
        """
        #把道路從影格中裁切出來
        road=frame[0:480,0:720] 


        fgmask = fgbg.apply(road)     #background
        fgmask = cv2.GaussianBlur(fgmask,(5,5),0) #GaussianBlue
        ret,fgmask = cv2.threshold(fgmask,120,255,cv2.THRESH_BINARY)   #Threshold
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)  #影像型態open處理
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_CLOSE, kernel) #影像型態close處理

        #獲得真實車輛的影像
        car = cv2.bitwise_and(road,road,mask = fgmask)

        #inverse車輛的遮罩
        fgmask_inv = cv2.bitwise_not(fgmask)

        #用白色當背景
        white=road.copy() 
        white[:,:]=255

        #白色背景減去車輛遮罩，變成黑色車子在白色背景移動
        road_withoutCar = cv2.bitwise_and(white,white,mask = fgmask_inv)

        #[黑色車子在白色背景]+[真實車輛的影像]
        whiteroad_car = cv2.add(road_withoutCar,car)

        #取得車子的輪廓
        image, contours, hierarchy = cv2.findContours(fgmask.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        #用線在車子輪廓上描出外框
        car_contour1=road.copy()
        car_contour1 = cv2.drawContours(car_contour1, contours, -1, (0,255,0), 3)


        car_contour2=road.copy()
        lock.acquire()
        global car_contour3
        car_contour3=road.copy()
        lock.release()
        car_contour4=road.copy()

        car1=road.copy()

        ##設定車輛追蹤範圍區
        startX1 = 150
        startX2 = 230
        endX = 500
        deltaDist = 50


        #設定車子進入哪個區域產生觸發，被捕捉影像
        bX1=190
        bX2=530
        bY1=120
        bY2=300
        cv2.line(car_contour3,(bX1,bY1),(bX1,bY2),color=(0,0,255),thickness=1)
        cv2.line(car_contour3,(bX2,bY1),(bX2,bY2),color=(0,0,255),thickness=1)
        cv2.line(car_contour3,(bX1,bY2),(bX2,bY2),color=(0,0,255),thickness=1)
      
        #當畫面中沒有車輛，就清空posList
        if len(contours)==0:
            posList=[]


        #對每一台車子做處理
        for i in range(len(contours)):
            cnt = contours[i]

            #找出每個車輛的位置和大小
            x,y,w,h = cv2.boundingRect(cnt)

            #取出面積
            area = cv2.contourArea(cnt)
          
            #用綠線描繪最接近車輛外型的輪廓
            epsilon = 0.1*cv2.arcLength(cnt,True)
            approx = cv2.approxPolyDP(cnt,epsilon,True)
            car_contour2 = cv2.drawContours(car_contour2, [approx], -1, (0,255,0), 3)

            lock.acquire()
            #用綠線描繪hull包圍車輛外型的輪廓-car_contour3
            hull = cv2.convexHull(cnt)
            car_contour3 = cv2.drawContours(car_contour3, [hull], -1, (200,150,100), 2)
            lock.release()

            #如果車輛進入觸發區，就做什麼事
            #if x<bX2 and x>bX1 and y<bY2 :   
                #car_num =car_num+1
                #即時畫出車輛的方形外框
                #car_contour4 = cv2.rectangle(car_contour4,(x,y),(x+w,y+h),(0,255,0),2)  #draw car box contour

                #將車輛方形外框剪裁入car1
                #car1=road[y:y+h,x:x+w]
                #cv2.imshow('car1',car1)
                #print(car_num)
          #把car1寫入檔案，檔名用車輛的計數
                #cv2.imwrite("car/"+str(car_num)+".png",car1)


            #用moment找出車輛質心 catteroid
            M = cv2.moments(cnt)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))



              
            #當車輛的位置進入範圍 >= endX and < startX1 表示由左到右
            if center[0] <= endX and center[0] >startX1:
                
                #只要進到x1-x2的範圍，就加入posList 、allCarPos，並在pictag設定尚未拍照
                if center[0] < startX2 :
                    allCarPos.append([(center[0],center[1])])
                    posList.append(len(allCarPos)-1)
                    pictag.append(0)
                    
                    
                else:
                    #若posList不為空
                    #設定最小距離為一個很大的數值
                    minDist = 65535
                    #設定預測自己在allCarPos裡的k是-1
                    kPred = -1
                    
                   #遍覽posList的數字k，這是車輛序號
                    for k in posList:
                        #用這數字到 allCarPos 去找最後一個座標
                        lastItem = allCarPos[k][len(allCarPos[k])-1] 
                        #計算各allCarPos最後一個座標跟自己的距離
                        dist = math.sqrt((center[0]-lastItem[0])**2+(center[1]-lastItem[1])**2)
                        
                        
                        #如果跟自己的距離<最小距離而且位置是在現在位置的右邊
                        if dist < minDist and lastItem[0] < center[0] :
                            #就把自己距離設定成最小距離
                            minDist = dist
                            #把kPred設成現在的k
                            kPred = k

                    #當最小距離 < 特定數值（車子前後影格變少於某門檻值)，確定就是這一台在前影格的位置
                    
                    if minDist < deltaDist:
                        #計算速度
                        #距離/時間
                        # 距離=minDist/scale/1000 km
                        # 時間=1/fps/60/60
                        velocity = (minDist/scale/1000)/(1/fps/60/60)
                        velocity = int(velocity)
                        velocity = str(velocity)
                        #把自己的位置寫到allCarPos的kPred
                        allCarPos[kPred].append((center[0],center[1]))
                      
                        #從上次位置到這次位置連線。用線相連
                        for i in range(1,len(allCarPos[kPred])):
                            cv2.line(car_contour3, allCarPos[kPred][i - 1], allCarPos[kPred][i], (0, 200, 105), 2)


                        #如果車輛進入觸發區，如果是，就拍一張影像
                        if x<bX2 and x>bX1 and y<bY2 :
                            #檢查在pictag的標籤是否Fale，代表尚未拍照
                            if pictag[kPred] == 0 :
                                #呈現速度
                                cv2.putText(car_contour3,velocity, (bX1, bY2), cv2.FONT_HERSHEY_SIMPLEX,3, (0, 0, 255), 3)
                                
                                #即時畫出車輛的方形外框，代表被拍
                                car_contour3 = cv2.rectangle(car_contour3,(x,y),(x+w,y+h),(255,255,255),5)
                                #將車輛方形外框剪裁入car1
                                car1=road[y:y+h,x:x+w]
                                carResize=cv2.resize(car1,(w*4,h*4), interpolation = cv2.INTER_CUBIC)
                                cv2.imshow('car2',carResize)
                                #print(car_num)
                                  
                                #把car1寫入檔案，檔名用車輛的計數
                                if int(velocity)>ExceedSpeed:
                                    cv2.imwrite(str(car_num)+".png",carResize)
                            
                                print("---")
                                print('由左至右')
                                print(car_num,":",velocity,"KM/H")
                                if int(velocity)>ExceedSpeed:
                                    print("!!!超速已拍照!!!")
                                print("---")

                                #已拍照註記
                                pictag[kPred] = 1   

                                #拍照序號+1
                                car_num_lock.acquire()
                                car_num =car_num+1
                                car_num_lock.release()
              

                      


            
            #自己的位置 < endX ，代表已經離開範圍了
            if center[0] > endX:
                
                #檢查posList是否為空   #若為空，代表自己已經不在車輛位置列表了，所以不做任何事情
                #若不是空，代表還在位置列表中
                if len(posList) > 0:
                    #若posList不為空
                    #設定最小距離為一個很大的數值
                    minDist = 65535
                    #設定預測自己在allCarPos裡的k是-1
                    kPred = -1
                    #遍覽posList的數字k
                    for k in posList:
                        
                        #用這數字到 allCarPos 去找最後一個座標
                        lastItem = allCarPos[k][len(allCarPos[k])-1] 
                        #計算跟自己的距離
                        dist = math.sqrt((center[0]-lastItem[0])**2+(center[1]-lastItem[1])**2)
                       
                        #如果跟自己的距離<最小距離而且之前的位置是在現在位置的右邊
                        if dist < minDist and lastItem[0] < center[0] :
                            #就把自己距離設定成最小距離
                            minDist = dist
                            #把kPred設成現在的k
                            kPred = k

                    #當最小距離 < 特定數值（車子前後影格變動小於某個門檻值)，就把自己位置寫入allCarPos
                    if minDist < deltaDist:
                        allCarPos[kPred].append((center[0],center[1]))

                    #把自己的位置kPred從posList中刪除，代表自己已經離開舞台
                    posList.remove(kPred)
                      

          

        if 0:       
            i=1
            for row in allCarPos:
                if len(row) > 5:
                    print(i,len(row))
                    i += 1
            print("\n")

        #調整waitkey 控制播放速度
        k = cv2.waitKey(1) & 0xff
        if k == 27:
            break          

                       
        WaitThreadBarrier.wait()

#################################
        
def RightToLeft_carTracking():

    fgbg = cv2.createBackgroundSubtractorMOG2()
    kernel = np.ones((21,21),np.uint8)
    #scale : 每公尺多少像素
    scale = 40
    allCarPos = []
    posList = []
    pictag = []
    global car_num
    #開始處理影格
    while(1):
        LoadFrameBarrier.wait()

        #獲知每秒多少影格frame per second
        global cap
        fps = cap.get(cv2.CAP_PROP_FPS)
        """
        #影格播完了，就播放下一個，直到video_index到達影片檔案的數量，就停止
        if frame is None:

            video_index += 1
            if video_index >= len(videofiles):
                break
            cap = cv2.VideoCapture(videofiles[ video_index ])
            ret, frame = cap.read()
        """
        #把道路從影格中裁切出來
        road=frame[0:480,0:720]


        fgmask = fgbg.apply(road)     #background
        fgmask = cv2.GaussianBlur(fgmask,(5,5),0) #GaussianBlue
        ret,fgmask = cv2.threshold(fgmask,120,255,cv2.THRESH_BINARY)   #Threshold
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)  #影像型態open處理
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_CLOSE, kernel) #影像型態close處理

        #獲得真實車輛的影像
        car = cv2.bitwise_and(road,road,mask = fgmask)

        #inverse車輛的遮罩
        fgmask_inv = cv2.bitwise_not(fgmask)

        #用白色當背景
        white=road.copy() 
        white[:,:]=255

        #白色背景減去車輛遮罩，變成黑色車子在白色背景移動
        road_withoutCar = cv2.bitwise_and(white,white,mask = fgmask_inv)

        #[黑色車子在白色背景]+[真實車輛的影像]
        whiteroad_car = cv2.add(road_withoutCar,car)

        #取得車子的輪廓
        image, contours, hierarchy = cv2.findContours(fgmask.copy(),cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        #用線在車子輪廓上描出外框
        car_contour1=road.copy()
        car_contour1 = cv2.drawContours(car_contour1, contours, -1, (0,255,0), 3)


        car_contour2=road.copy()
        lock.acquire()
        global car_contour3
        car_contour3=road.copy()
        lock.release()
        car_contour4=road.copy()

        car1=road.copy()

        ##設定車輛追蹤範圍區
        startX1 = 500
        startX2 = 350
        endX = 5
        deltaDist = 50


        #設定車子進入哪個區域產生觸發，被捕捉影像
        bX1=190
        bX2=530
        bY1=120
        bY2=300
        cv2.line(car_contour3,(bX1,bY1),(bX1,bY2),color=(0,0,255),thickness=1)
        cv2.line(car_contour3,(bX2,bY1),(bX2,bY2),color=(0,0,255),thickness=1)
        cv2.line(car_contour3,(bX1,bY2),(bX2,bY2),color=(0,0,255),thickness=1)
      
        #當畫面中沒有車輛，就清空posList
        if len(contours)==0:
            posList=[]


        #對每一台車子做處理
        for i in range(len(contours)):
            cnt = contours[i]

            #找出每個車輛的位置和大小
            x,y,w,h = cv2.boundingRect(cnt)

            #取出面積
            area = cv2.contourArea(cnt)
          
            #用綠線描繪最接近車輛外型的輪廓
            epsilon = 0.1*cv2.arcLength(cnt,True)
            approx = cv2.approxPolyDP(cnt,epsilon,True)
            car_contour2 = cv2.drawContours(car_contour2, [approx], -1, (0,255,0), 3)

            lock.acquire()
            #用綠線描繪hull包圍車輛外型的輪廓-car_contour3
            hull = cv2.convexHull(cnt)
            car_contour3 = cv2.drawContours(car_contour3, [hull], -1, (200,150,100), 2)
            lock.release()

            #如果車輛進入觸發區，就做什麼事
            #if x<bX2 and x>bX1 and y<bY2 :   
                #car_num =car_num+1
                #即時畫出車輛的方形外框
                #car_contour4 = cv2.rectangle(car_contour4,(x,y),(x+w,y+h),(0,255,0),2)  #draw car box contour

                #將車輛方形外框剪裁入car1
                #car1=road[y:y+h,x:x+w]
                #cv2.imshow('car1',car1)
                #print(car_num)
          #把car1寫入檔案，檔名用車輛的計數
                #cv2.imwrite("car/"+str(car_num)+".png",car1)


            #用moment找出車輛質心 catteroid
            M = cv2.moments(cnt)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))



              
            #當車輛的位置進入範圍 >= endX and < startX1 表示由右到左
            if center[0] >= endX and center[0] < startX1:
                #只要進到x1-x2的範圍，就加入posList 、allCarPos，並在pictag設定尚未拍照
                if center[0] > startX2 :
                    allCarPos.append([(center[0],center[1])])
                    posList.append(len(allCarPos)-1)
                    pictag.append(0)

                else:
                    #若posList不為空
                    #設定最小距離為一個很大的數值
                    minDist = 65535
                    #設定預測自己在allCarPos裡的k是-1
                    kPred = -1

                   #遍覽posList的數字k，這是車輛序號
                    for k in posList:
                        #用這數字到 allCarPos 去找最後一個座標
                        lastItem = allCarPos[k][len(allCarPos[k])-1] 
                        #計算各allCarPos最後一個座標跟自己的距離
                        dist = math.sqrt((center[0]-lastItem[0])**2+(center[1]-lastItem[1])**2)
                      
                        #如果跟自己的距離<最小距離而且位置是在現在位置的左邊
                        if dist < minDist and lastItem[0] > center[0] :
                            #就把自己距離設定成最小距離
                            minDist = dist
                            #把kPred設成現在的k
                            kPred = k
                    
                    #當最小距離 < 特定數值（車子前後影格變少於某門檻值)，確定就是這一台在前影格的位置
                    if minDist < deltaDist:
                        #計算速度
                        #距離/時間
                        # 距離=minDist/scale/1000 km
                        # 時間=1/fps/60/60
                        velocity = (minDist/scale/1000)/(1/fps/60/60)
                        velocity = int(velocity)
                        velocity = str(velocity)
                        #把自己的位置寫到allCarPos的kPred
                        allCarPos[kPred].append((center[0],center[1]))
                      
                        #從上次位置到這次位置連線。用線相連
                        for i in range(1,len(allCarPos[kPred])):
                            cv2.line(car_contour3, allCarPos[kPred][i - 1], allCarPos[kPred][i], (0, 200, 105), 2)


                        #如果車輛進入觸發區，如果是，就拍一張影像
                        if x<bX2 and x>bX1 and y<bY2 :
                            #檢查在pictag的標籤是否Fale，代表尚未拍照
                            if pictag[kPred] == 0 :
                                #呈現速度
                                cv2.putText(car_contour3,velocity, (bX1, bY2), cv2.FONT_HERSHEY_SIMPLEX,3, (0, 0, 255), 3)

                                #即時畫出車輛的方形外框，代表被拍
                                car_contour3 = cv2.rectangle(car_contour3,(x,y),(x+w,y+h),(255,255,255),5)
                                #將車輛方形外框剪裁入car1
                                car1=road[y:y+h,x:x+w]
                                carResize=cv2.resize(car1,(w*4,h*4), interpolation = cv2.INTER_CUBIC)
                                cv2.imshow('car1',carResize)
                                #print(car_num)
                                  
                                #把car1寫入檔案，檔名用車輛的計數
                                if int(velocity)>ExceedSpeed:
                                    cv2.imwrite(str(car_num)+".png",carResize)
                            
                                print("---")
                                print('由右至左')
                                print(car_num,":",velocity,"KM/H")
                                if int(velocity)>ExceedSpeed:
                                    print("!!!超速已拍照!!!")
                                print("---")

                                #已拍照註記
                                pictag[kPred] = 1   

                                #拍照序號+1
                                car_num_lock.acquire()
                                car_num =car_num+1
                                car_num_lock.release()

              

                      



            #自己的位置 < endX ，代表已經離開範圍了
            if center[0] < endX:
                
                #檢查posList是否為空   #若為空，代表自己已經不在車輛位置列表了，所以不做任何事情
                #若不是空，代表還在位置列表中
                if len(posList) > 0:
                    #若posList不為空
                    #設定最小距離為一個很大的數值
                    minDist = 65535
                    #設定預測自己在allCarPos裡的k是-1
                    kPred = -1
                    #遍覽posList的數字k
                    for k in posList:
                        #用這數字到 allCarPos 去找最後一個座標
                        lastItem = allCarPos[k][len(allCarPos[k])-1] 
                        #計算跟自己的距離
                        dist = math.sqrt((center[0]-lastItem[0])**2+(center[1]-lastItem[1])**2)
                      
                        #如果跟自己的距離<最小距離而且之前的位置是在現在位置的右邊
                        if dist < minDist and lastItem[0] > center[0] :
                            #就把自己距離設定成最小距離
                            minDist = dist
                            #把kPred設成現在的k
                            kPred = k

                    #當最小距離 < 特定數值（車子前後影格變動小於某個門檻值)，就把自己位置寫入allCarPos
                    if minDist < deltaDist:
                        allCarPos[kPred].append((center[0],center[1]))

                    #把自己的位置kPred從posList中刪除，代表自己已經離開舞台
                    posList.remove(kPred)
                      

          

        if 0:       
            i=1
            for row in allCarPos:
                if len(row) > 5:
                    print(i,len(row))
                    i += 1
            print("\n")

        #調整waitkey 控制播放速度
        k = cv2.waitKey(1) & 0xff
        if k == 27:
            break

        WaitThreadBarrier.wait()


#Main
if __name__ == '__main__':

    global cap
    cap = cv2.VideoCapture('testtest1.avi')
    ret, frame = cap.read()
    
    RightThread = threading.Thread(name='RightToLeft_carTracking', target=RightToLeft_carTracking, args=())
    RightThread.setDaemon(True)
    RightThread.start()

    LeftThread = threading.Thread(name='LeftToRight_carTracking', target=LeftToRight_carTracking, args=())
    LeftThread.setDaemon(True)
    LeftThread.start()

    while(1):
        ret, frame = cap.read()
        if ret==True:  
            LoadFrameBarrier.wait()

            #等待Thread影像處理
            WaitThreadBarrier.wait()
            
            cv2.imshow('car_contour3',car_contour3)
        else:
            k = 27

        #調整waitkey 控制播放速度
        k = cv2.waitKey(1) & 0xff
        if k == 27:
            print('release')
            break

    cap.release()
    cv2.destroyAllWindows()



