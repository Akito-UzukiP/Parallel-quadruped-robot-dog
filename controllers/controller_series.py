"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
import keyboard
import numpy as np
from controller.robot import Robot
from controller import wb
#机器人常数
legtype='series'
motor0_radius = 0.01
pi = math.pi
l1 =0.1
l2 = 0.1
l3 = 0.15
l4 = 0.12
l5 = 0.03#末端与l3-l4连杆转轴的距离
l7 = 0.1
l8 = 0.1
#!!!!注意!!!!!wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
#step_curve的xyz是在机体参考系的,与脚参考系不同！
#l1 l2 与夹角
def cosine_law_length(l1,l2,theta):
    return math.sqrt(l1*l1+l2*l2-2*l1*l2*math.cos(theta))
#计算l3对角
def cosine_law_theta(l1,l2,l3):
    return math.acos((l1*l1+l2*l2-l3*l3)/(2*l1*l2))
def my_atan(x,y):

    if x == 0:
        theta = pi/2
    else:
        theta = math.atan(y/x)
    if theta < 0:
        return theta+math.pi
    return theta
def ik(position,type='left'):#type='left'为左侧坐标系,type='right'为右侧 统一一下以左侧坐标系为正，把右侧给翻过来
    x = position[0]
    y_ = position[1]
    theta3=0

    #调用了全局变量转轴半径
    z = position[2]
    y = math.sqrt(z*z+y_*y_-motor0_radius*motor0_radius)
    if type =='left':
        theta1 = my_atan(motor0_radius,y)+my_atan(y_,z)-pi/2
    else:
        theta1 = pi/2 + my_atan(motor0_radius,y) - my_atan(y_,z)
    theta0 = my_atan(x,y)
    if theta1 > pi/2:
        theta1 -=  pi
    if legtype=='quad':
        k = math.sqrt(x * x + y * y)
        alpha = math.acos((k * k + l1 * l1 - l3 * l3) / 2 / l1 / k)
        beta = math.acos((l1 * l1 + l3 * l3 - k * k) / 2 / l1 / l3)
    #gamma = math.pi + alpha - beta
        theta3 = theta0 - alpha
        gamma = math.pi +theta3 - beta
        x2 = x - l5 * math.cos(gamma)
        y2 = y - l5 * math.sin(gamma)

        theta02 = my_atan(x2,y2)
        z2 = math.sqrt(x2 * x2 + y2 * y2)
        alpha2 = math.acos((z2*z2+l1*l1-l4*l4)/2/l1/z2)
        theta2 = theta02 + alpha2
        if (type == 'right'):
            return -theta1, theta2, theta3,
        return theta1, theta2, theta3  # 连杆电机21,1--》根部电机
    elif legtype == 'series':
        theta3 = theta1
        alpha = my_atan(x,y)
        l6 = math.sqrt(x*x+y*y)
        theta2 = pi-cosine_law_theta(l7,l8,l6)
        theta1 = alpha-cosine_law_theta(l7,l6,l8)
        print(theta3)
        if (type == 'right'):
            return -theta3, theta1, theta2,
        return theta3, theta1, theta2  # 连杆电机21,1--》根部电机
    return 0,0,0
def fk(theta,type='left'):
    #theta[0]是根部电机,1是短连杆，2是长连杆
    alpha = theta[1]-theta[2] #夹角
    lbc = cosine_law_length(l1,l2,alpha)
   # theta_abc = cosine_law_theta(l1,lbc,l2)
   # theta_dbc = cosine_law_tehta((l3-l5),lbc,l4)
    theta_b = cosine_law_theta(l1,lbc,l2)+cosine_law_theta((l3-l5),lbc,l4)
    lap = cosine_law_length(l1,l3,theta_b)
    beta = cosine_law_theta(l1,lap,l3)
    gamma = theta[2]+beta
    p = [lap*math.cos(gamma),lap*math.sin(gamma),0]
    if type == 'left':
        p[2] = p[1]*math.sin(theta[0])+motor0_radius*math.cos(theta[0])
        p[1] = p[1]*math.cos(theta[0])-motor0_radius*math.sin(theta[0])
    else:
        p[2] = p[1]*math.sin(theta[0]) - motor0_radius*math.cos(theta[0])
        p[1] = p[1]*math.cos(theta[0]) + motor0_radius*math.sin(theta[0])
    return p
def step_curve(start_point,end_point,progess,height=0.02):

    x_s,y_s,z_s = start_point
    x_e,y_e,z_e = end_point
    sigma = 2*pi*progess
    x_c = (x_e - x_s) * (sigma - math.sin(sigma)) / (2 * pi) + x_s
    y_c = (y_e - y_s) * (sigma - math.sin(sigma)) / (2 * pi) + y_s
    #h_c是不用转换的
    h_c = -height *(1-math.cos(sigma))/2 + z_s
    return x_c, y_c, h_c
def straight_curve(start_point, end_point,progress):
    return ((np.array(end_point)-np.array(start_point)) * progress + np.array(start_point)).tolist()
#腿参考系
def get_positions():
    angles_ = []
    for i in motors_:
        angles_.append(i.getTargetPosition())
    angles = np.array(angles_).reshape((4,3))
    positions = []
    for i in angles:
        positions.append(fk(i))
    return positions
def rpy2angle(rpy,height):
    current_position = get_positions()
    x_len = 0.13*2
    y_len = 0.06*2
    roll = rpy[0]#绕x轴转动的角度
    pitch = rpy[1]#绕y轴转动的
    yaw = rpy[2]#无用
    #顺时针转动 如果roll为正则左倾,左边应该高右边应该低
    #如果pitch为正则前倾，应该前面高后面低
    cdz_fb = (current_position[1][1] + current_position[3][1] - current_position[0][1] - current_position[2][1])/4
    cdz_lr = (current_position[0][1] + current_position[1][1] - current_position[2][1] - current_position[3][1])/4
    dz_lr = y_len*math.tan(roll)/2
    dz_fb = x_len*math.tan(pitch)/2
    delta0 = cdz_lr - cdz_fb  - dz_lr + dz_fb
    delta1 = cdz_lr + cdz_fb  - dz_lr - dz_fb
    delta2 = -cdz_lr - cdz_fb + dz_lr + dz_fb
    delta3 = -cdz_lr + cdz_fb + dz_lr - dz_fb
    return  [[0,height+delta0,0],[0,height+delta1,0],[0,height+delta2,0],[0,height+delta3,0]]
def smooth_to_angle(targets,steps):
    current_ = []
    for i in motors_:
        temp =  i.getTargetPosition() % (2*pi)
        if temp>1.5*pi:
            temp-=2*pi
        current_.append(temp)
    current = current_
    #这样保证了target是正的0~2pi位置
    #按理来说 current应该在-pi~pi上 也应该只允许在这个范围内 不允许转过一圈
    #

    target = np.mod(targets , (2*pi))
    for i in range(len(target)):
        if target[i]>1.5*pi:
            target[i] -=2*pi
            #print(target[i])

    #print(target)
    #print(current," ",target)
    for i in range(steps):
        for j in range(len(motors_)):
            motors_[j].setPosition(current_[j]+(target[j]-current[j])*i/steps)
        robot.step()
#传入的应该是4x3维,左前左后右前右后顺序的脚坐标系位置
def smooth_to_position(positions,steps):
    targets = []
    targets.append( list( ik(positions[0],type='left')))
    targets.append( list( ik(positions[1], type='left')))
    targets.append( list( ik(positions[2], type='right')))
    targets.append( list( ik(positions[3], type='right')))
    #print(targets)

    targets = [item for list in targets for item in list]
    #print(targets)
    smooth_to_angle(targets,steps)
def my_set_velocity(motor,velocity):
    current = motor.getTargetPosition()
    motor.setPosition(current+velocity/math.pi/32)

def simple_walk(yy,tt,period,length):
    if tt> period:
        tt = tt % period
    if tt > period/4*3:
        xx,zz,yy = step_curve((-length/2,motor0_radius,yy),(length/2,motor0_radius,yy),(tt/period-0.75)/0.25)
        #print(xx," ",yy," ",zz)
        #k = (tt-period*(7/8))/(period/4) *pi
        #xx = -0.05+length*math.sin(k)
        #yy = yy-0.5*length*math.cos(k)
    else:
        xx ,zz,yy= straight_curve((length/2,motor0_radius,yy), (-length/2,motor0_radius,yy), tt/(period*0.75))
        #xx = length/2 - length*(tt)/(period*(3/4))
    return xx,yy,0
def walk(timeStep,height = 0.13,length = 0.05, period = 32):
    t1 = timeStep % period
    t2 = t1+  0.25 * period
    t3 = t1 + 0.5 *  period
    t4 = t1 + 0.75 * period
    theta0, theta1,theta2 = ik(simple_walk(height, t1, period, length))
    #print(simple_walk(height,t1,period,length)," 1")
    #print(theta2, " ",theta1," ",theta0," 2")
    theta3, theta4,theta5 = ik(simple_walk(height, t2, period, length))
    theta6, theta7,theta8 = ik(simple_walk(height, t3, period, length), type='right')
    theta9, theta10, theta11= ik(simple_walk(height, t4, period, length), type='right')

    pass
    motors_[0].setPosition(theta0)
    motors_[1].setPosition(theta1)
    motors_[2].setPosition(theta2)
    motors_[3].setPosition(theta3)
    motors_[4].setPosition(theta4)
    motors_[5].setPosition(theta5)
    motors_[6].setPosition(theta6)
    motors_[7].setPosition(theta7)
    motors_[8].setPosition(theta8)
    motors_[9].setPosition(theta9)
    motors_[10].setPosition(theta10)
    motors_[11].setPosition(theta11)

def simple_trot(yy,tt,period,length):
    if tt> period:
        tt = tt % period
    if tt > period/2:
        xx, zz, yy = step_curve((-length / 2, motor0_radius, yy), (length / 2, motor0_radius, yy), (tt / period - 0.5) / 0.5)
    else:
        xx,zz,yy = straight_curve((length / 2, motor0_radius, yy), (-length / 2, motor0_radius, yy), tt / (period * 0.5))
        #xx = length/2 - length*(tt)/(period*(1/2))
    return xx,yy,0
def trot(timeStep,height = 0.13,length = 0.05, period = 16):
    t1 = timeStep % period
    t2 = t1+  0.5 * period
    theta0, theta1,theta2 = ik(simple_trot(height, t1, period, length))
    theta3, theta4,theta5 = ik(simple_trot(height, t2, period, length))
    theta6, theta7,theta8 = ik(simple_trot(height, t2, period, length), type='right')
    theta9, theta10,theta11 = ik(simple_trot(height, t1, period, length), type='right')

    pass
    motors_[0].setPosition(theta0)
    motors_[1].setPosition(theta1)
    motors_[2].setPosition(theta2)
    motors_[3].setPosition(theta3)
    motors_[4].setPosition(theta4)
    motors_[5].setPosition(theta5)
    motors_[6].setPosition(theta6)
    motors_[7].setPosition(theta7)
    motors_[8].setPosition(theta8)
    motors_[9].setPosition(theta9)
    motors_[10].setPosition(theta10)
    motors_[11].setPosition(theta11)
#丑陋不堪的代码 明天重写
def turn_inplace_one_step(theta,period,height=0.13):
    _x = 0.13
    _y = 0.06
    _r = math.sqrt(_x*_x+_y*_y)
    #R是后位置到前位置的转换矩阵
    R = np.array([[math.cos(theta),-math.sin(theta),0],
         [math.sin(theta),math.cos(theta),0],
         [0,0,1]])
    legs = np.array([[_x, _y, 0],[-_x, _y, 0],[_x, -_y, 0],[-_x, -_y, 0]])
    #1是向前的
    legs_n_1 = R.dot(legs.T).T
    #3是跟上的
    movements1 = legs_n_1 - legs + np.array([0,0,height])
    legs_n_2 = R.T.dot(legs.T).T
    movements2 = legs_n_2 - legs + np.array([0, 0, height])
    print(movements1,"\n",movements2,'\n',theta)
    if theta > 0:
        print('leftTurn')
        for j in range(period):
            k = j/period
            x,y,z = step_curve((0,0,height),movements1[0],k)
            theta0,theta1,theta2 = ik((x, z, y))
            motors_[0].setPosition(theta0)
            motors_[1].setPosition(theta1)
            motors_[2].setPosition(theta2)
            x,y,z = straight_curve((0,0,height),movements2[1],k)
            theta0,theta1,theta2 = ik((x, z, y))
            motors_[3].setPosition(theta0)
            motors_[4].setPosition(theta1)
            motors_[5].setPosition(theta2)
            x,y,z = straight_curve((0,0,height),movements2[2],k)
            theta0,theta1,theta2 = ik((x, z, y), type='right')
            motors_[6].setPosition(theta0)
            motors_[7].setPosition(theta1)
            motors_[8].setPosition(theta2)
            x,y,z = step_curve((0,0,height),movements1[3],k)
            theta0,theta1,theta2 = ik((x, z, y), type='right')
            motors_[9].setPosition(theta0)
            motors_[10].setPosition(theta1)
            motors_[11].setPosition(theta2)
            robot.step()
        pass
        for j in range(period):
            k = j/period
            x,y,z = straight_curve(movements1[0],(0,0,height),k)
            theta0,theta1,theta2 = ik((x, z, y))
            motors_[0].setPosition(theta0)
            motors_[1].setPosition(theta1)
            motors_[2].setPosition(theta2)
            x,y,z = step_curve(movements2[1],(0,0,height),k)
            theta0,theta1,theta2 = ik((x, z, y))
            motors_[3].setPosition(theta0)
            motors_[4].setPosition(theta1)
            motors_[5].setPosition(theta2)
            x,y,z = step_curve(movements2[2],(0,0,height),k)
            theta0,theta1,theta2 = ik((x, z, y), type='right')
            motors_[6].setPosition(theta0)
            motors_[7].setPosition(theta1)
            motors_[8].setPosition(theta2)
            x,y,z = straight_curve(movements1[3],(0,0,height),k)
            theta0,theta1,theta2 = ik((x, z, y), type='right')
            motors_[9].setPosition(theta0)
            motors_[10].setPosition(theta1)
            motors_[11].setPosition(theta2)
            robot.step()
    else:
        print('rightTurn')
        for j in range(period):
            k = j/period
            x,y,z = straight_curve((0,0,height),movements2[0],k)
            theta0,theta1,theta2 = ik((x, z, y))
            motors_[0].setPosition(theta0)
            motors_[1].setPosition(theta1)
            motors_[2].setPosition(theta2)
            x,y,z = step_curve((0,0,height),movements1[1],k)
            theta0,theta1,theta2 = ik((x, z, y))
            motors_[3].setPosition(theta0)
            motors_[4].setPosition(theta1)
            motors_[5].setPosition(theta2)
            x,y,z = step_curve((0,0,height),movements1[2],k)
            theta0,theta1,theta2 = ik((x, z, y), type='right')
            motors_[6].setPosition(theta0)
            motors_[7].setPosition(theta1)
            motors_[8].setPosition(theta2)
            x,y,z = straight_curve((0,0,height),movements2[3],k)
            theta0,theta1,theta2 = ik((x, z, y), type='right')
            motors_[9].setPosition(theta0)
            motors_[10].setPosition(theta1)
            motors_[11].setPosition(theta2)
            robot.step()
        pass
        for j in range(period):
            k = j/period
            x,y,z = step_curve(movements2[0],(0,0,height),k)
            theta0,theta1,theta2 = ik((x, z, y))
            motors_[0].setPosition(theta0)
            motors_[1].setPosition(theta1)
            motors_[2].setPosition(theta2)
            x,y,z = straight_curve(movements1[1],(0,0,height),k)
            theta0,theta1,theta2 = ik((x, z, y))
            motors_[3].setPosition(theta0)
            motors_[4].setPosition(theta1)
            motors_[5].setPosition(theta2)
            x,y,z = straight_curve(movements1[2],(0,0,height),k)
            theta0,theta1,theta2 = ik((x, z, y), type='right')
            motors_[6].setPosition(theta0)
            motors_[7].setPosition(theta1)
            motors_[8].setPosition(theta2)
            x,y,z = step_curve(movements2[3],(0,0,height),k)
            theta0,theta1,theta2 = ik((x, z, y), type='right')
            motors_[9].setPosition(theta0)
            motors_[10].setPosition(theta1)
            motors_[11].setPosition(theta2)
            robot.step()


#计算在转向theta后在各个脚的坐标系下的变化量，以numpy.array形式返回xyz坐标(各个脚坐标系下的),4组
def radius_turning_calculation(radius,theta):
    R01 = np.array([[math.cos(theta),-math.sin(theta),0],
                    [math.sin(theta),math.cos(theta),0],
                   [0,0,1]])#转向后坐标系相对原坐标系的转换矩阵R01
    r_ = np.array([[0.13,0.06,0],
                    [-0.13,0.06,0],
                    [0.13,-0.06,0],
                    [-0.13,-0.06,0]])#相对狗中心的各点坐标
    r = r_ - np.array([0,radius,0])#相对转向半径中心的坐标，R为正时向左转
    #radius = np.sqrt((refer_to_radius_center*refer_to_radius_center).sum(axis=1))#相对转向半径中心各点的半径
    r1 = R01.dot(r.T).T
    return r1-r
def turn_radius_one_step(radius,theta,period,left_right='back',front_back='right',height=0.13):
    radius = abs(radius)
    theta = abs(theta)
    alter = False
    if(left_right == 'right') & (front_back == 'front'):
        alter = True
        radius = radius
        theta = -theta
    elif(left_right == 'left') & (front_back == 'front'):
        radius = -radius
        theta = theta
    elif(left_right == 'left') & (front_back == 'back'):
        alter = True
        radius = -radius
        theta = -theta
    else:
        radius = radius
        theta = theta
    movements13 = radius_turning_calculation(radius,theta)
    movements24 = radius_turning_calculation(radius,-theta)
    movements13 = movements13 + np.array([0, 0, height])
    movements24 = movements24 + np.array([0, 0, height])
    if not alter:
        for j in range(period):
            k = j/period
            x,y,z = step_curve((0,0,height),movements13[0],k)
            theta0,theta1,theta2 = ik((x, z, y))
            motors_[0].setPosition(theta0)
            motors_[1].setPosition(theta1)
            motors_[2].setPosition(theta2)
            x,y,z = straight_curve((0,0,height),movements24[1],k)
            theta0,theta1,theta2 = ik((x, z, y))
            motors_[3].setPosition(theta0)
            motors_[4].setPosition(theta1)
            motors_[5].setPosition(theta2)
            x,y,z = straight_curve((0,0,height),movements24[2],k)
            theta0, theta1, theta2 = ik((x, z, y), type='right')
            motors_[6].setPosition(theta0)
            motors_[7].setPosition(theta1)
            motors_[8].setPosition(theta2)
            x,y,z = step_curve((0,0,height),movements13[3],k)
            theta0,theta1,theta2= ik((x, z, y), type='right')
            motors_[9].setPosition(theta0)
            motors_[10].setPosition(theta1)
            motors_[11].setPosition(theta2)
            robot.step()
        pass
        for j in range(period):
            k = j/period
            x,y,z = straight_curve(movements13[0],(0,0,height),k)
            theta0,theta1,theta2 = ik((x, z, y))
            motors_[0].setPosition(theta0)
            motors_[1].setPosition(theta1)
            motors_[2].setPosition(theta2)
            x,y,z = step_curve(movements24[1],(0,0,height),k)
            theta0, theta1, theta2= ik((x, z, y))
            motors_[3].setPosition(theta0)
            motors_[4].setPosition(theta1)
            motors_[5].setPosition(theta2)
            x,y,z = step_curve(movements24[2],(0,0,height),k)
            theta0,theta1,theta2= ik((x, z, y), type='right')
            motors_[6].setPosition(theta0)
            motors_[7].setPosition(theta1)
            motors_[8].setPosition(theta2)
            x,y,z = straight_curve(movements13[3],(0,0,height),k)
            theta0, theta1, theta2 = ik((x, z, y), type='right')
            motors_[9].setPosition(theta0)
            motors_[10].setPosition(theta1)
            motors_[11].setPosition(theta2)
            robot.step()
    else:
        for j in range(period):
            k = j / period
            x, y, z = straight_curve((0, 0, height), movements24[0], k)
            theta0, theta1, theta2 = ik((x, z, y))
            motors_[0].setPosition(theta0)
            motors_[1].setPosition(theta1)
            motors_[2].setPosition(theta2)
            x, y, z = step_curve((0, 0, height), movements13[1], k)
            theta0, theta1, theta2 = ik((x, z, y))
            motors_[3].setPosition(theta0)
            motors_[4].setPosition(theta1)
            motors_[5].setPosition(theta2)
            x, y, z = step_curve((0, 0, height), movements13[2], k)
            theta0, theta1, theta2 = ik((x, z, y), type='right')
            motors_[6].setPosition(theta0)
            motors_[7].setPosition(theta1)
            motors_[8].setPosition(theta2)
            x, y, z = straight_curve((0, 0, height), movements24[3], k)
            theta0, theta1, theta2 = ik((x, z, y), type='right')
            motors_[9].setPosition(theta0)
            motors_[10].setPosition(theta1)
            motors_[11].setPosition(theta2)
            robot.step()
        pass
        for j in range(period):
            k = j / period
            x, y, z = step_curve(movements24[0], (0, 0, height), k)
            theta0, theta1, theta2 = ik((x, z, y))
            motors_[0].setPosition(theta0)
            motors_[1].setPosition(theta1)
            motors_[2].setPosition(theta2)
            x, y, z = straight_curve(movements13[1], (0, 0, height), k)
            theta0, theta1, theta2 = ik((x, z, y))
            motors_[3].setPosition(theta0)
            motors_[4].setPosition(theta1)
            motors_[5].setPosition(theta2)
            x, y, z = straight_curve(movements13[2], (0, 0, height), k)
            theta0, theta1, theta2 = ik((x, z, y), type='right')
            motors_[6].setPosition(theta0)
            motors_[7].setPosition(theta1)
            motors_[8].setPosition(theta2)
            x, y, z = step_curve(movements24[3], (0, 0, height), k)
            theta0, theta1, theta2 = ik((x, z, y), type='right')
            motors_[9].setPosition(theta0)
            motors_[10].setPosition(theta1)
            motors_[11].setPosition(theta2)
            robot.step()

#把roll pitch yaw根据yaw做一个旋转变换

##########主代码部分##########
robot = Robot()


timestep = int(robot.getBasicTimeStep())

#电机命名顺序:左前左后右前右后
motor_names=['motor1','motor2','motor3','motor4','motor5','motor6','motor7','motor8','motor9','motor10','motor11','motor12']
motors = {}
motors_ = []
# gyro = robot.getDevice('gyro')
# gyro.enable(32)
# imu = robot.getDevice('IMU')
# print(imu)
# imu.enable(32)
# print(gyro)
for i in motor_names:
    print(robot.getDevice(i))
    motors[i] = robot.getDevice(i)
    motors_.append(robot.getDevice((i)))



t=1

#简单步态的部分设定
x = -0.1
y = 0.13
r = 0.05
period = 16
smooth_to_position([[0, 0.13, 0],
                    [0, 0.13, 0],
                    [0, 0.13, 0],
                    [0, 0.13, 0]],16)

#smooth_to_angle([0.25*pi,0.25*pi,0.5*pi,
  #                        0.25*pi,0.25*pi,0.5*pi,
    #                      0.25*pi,0.25*pi,0.5*pi,
                #          0.25*pi,0.25*pi,0.5*pi],64)
t_=0


while robot.step(timestep) != -1:
    # rpy = imu.getRollPitchYaw()
    # print(rpy)
    t_+=1
    t = t_ % 32
    pass
    if keyboard.is_pressed("w"):
        if keyboard.is_pressed('a'):
            turn_radius_one_step(0.5,pi/16,8,front_back='front',left_right='left')
        elif keyboard.is_pressed(('d')):
            turn_radius_one_step(0.5,pi/16,8,front_back='front',left_right='right')
        else:
            #trot(-t,length=0.1)
            walk(-t,length=0.1)
    elif keyboard.is_pressed("s"):
        if keyboard.is_pressed('a'):
            turn_radius_one_step(0.5,pi/16,8,front_back='back',left_right='left')
        elif keyboard.is_pressed(('d')):
            turn_radius_one_step(0.5,pi/16,8,front_back='back',left_right='right')
        else:
            trot(t,length=0.1)
    elif keyboard.is_pressed('a'):
        turn_inplace_one_step(pi/8,8)
    elif keyboard.is_pressed('d'):
        turn_inplace_one_step(-pi/8,8)
    
