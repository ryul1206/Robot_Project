import cv2
import numpy as np 
import matplotlib.pyplot as plt
from scipy import ndimage
import math









src = cv2.imread('Map4.png',1)

gray_image = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
_ , binary_image = cv2.threshold(gray_image,127,255,cv2.THRESH_BINARY)

height, width = src.shape[:2]


xs = 630
ys = 315
xg = 110
yg = 80

 
U_att=np.zeros((height,width))

for i in range(height):
    for j in range(width):
        
        rg = np.sqrt((i-yg)**2+(j-xg)**2)
        U_att[i][j] = 0.5*rg**2
      
        
##plt.contour(U_att,40)
##plt.imshow(src)


# type change binary_image

binary_image = (binary_image)/255
binary_image = binary_image.astype(np.uint8)



# Distance of influence of the object
Di = 10
D = ndimage.distance_transform_edt(binary_image)

U_rep=np.zeros((height,width))



for i in range(height):
    for j in range(width):
        
        rd = D[i][j]
        
        if rd <= Di:
            U_rep[i][j]= 0.5*(rd-Di)**2
            
        else:
            U_rep[i][j] = 0 
                
            
##plt.contour(U_rep,5)


k_s=2
U_sum = (U_rep)/(U_rep.max())+(k_s)*(U_att)/(U_att.max())
                
plt.contour(U_sum,30)                
            
x= xs
y= ys

last = U_sum[y-1][x-1]

dis=np.zeros((3,3))


#pose Matrix using matlab spy function 


pose = np.zeros((height,width))

pose[ys][xs] = 1 
pose[yg][xg] = 1 
##plt.spy(pose)
## 여기부터 고치면        
while (x != xg) or (y != yg):
    
    dis[0][0]=U_sum[y-1][x-1]
    dis[0][1]=U_sum[y-1][x]
    dis[0][2]=U_sum[y-1][x+1]
    
    dis[1][0]=U_sum[y][x-1]
    dis[1][1]=U_sum[y][x]
    dis[1][2]=U_sum[y][x+1]
    
    dis[2][0]=U_sum[y+1][x-1]
    dis[2][1]=U_sum[y+1][x]
    dis[2][2]=U_sum[y+1][x+1]
    
    
    #m = dis.min()
    #r,c = np.where(dis==m)
    index = np.argmin(dis)
    r, c = divmod(index, 3)
    
    
    U_sum[y][x] = math.inf
    
    y= y-1+r
    x= x-1+c
    


    pose[y][x] = 1
    
    
plt.spy(pose)    
    

    
    
    
    
    
    



    
    
    
    
    










