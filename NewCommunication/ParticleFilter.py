import math
import random
import logging
import numpy as np
from hokuyolx import HokuyoLX
import time
import matplotlib.pyplot as plt
# Dimensions of the playing field
WORLD_X = 3000
WORLD_Y = 2000
INT_MAX = 99999990
mark_r = 40
#landmarks = [[WORLD_X/2, 0.0], [WORLD_X+mark_r, WORLD_Y], [-mark_r, WORLD_Y]]  # position of 4 landmarks in (x, y) format.
landmarks = [[-mark_r, WORLD_Y/2.], [WORLD_X+mark_r,WORLD_Y + mark_r], [WORLD_X + mark_r, - mark_r]]
#landmarks = [[0,1000],[1000,0],[1000,1000]]
# Noises
distance_noise = 30.0  # Noise parameter: should be included in move function.
angle_noise = 0.03#0.01  # Noise parameter: should be included in move function.
sense_noise = 40
particle_number = 3000  # Number of Particles

# ------------------------------------------------
# 
# this is the robot class
#
start_position = [170, 170, 0]  # should be two types and not [0,0,0]

# Support Functions


def ideal(x,y):
    answer = []
    for i in landmarks:
        answer.append(((x - i[0])**2 + (y - i[1])**2)**0.5+0*random.gauss(0, distance_noise))
    return answer

def MarkGaus(x,sigma=sense_noise):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return math.exp(- ((x) ** 2) / (sigma ** 2) / 2.0) / math.sqrt(2.0 * math.pi * (sigma ** 2))




class Robot:
    # --------
    # init:
    #    creates robot and initializes location/orientation
    #

    def __init__(self,a=start_position[0],b=start_position[1],ang = start_position[2]):
        # add gausssian
        self.x = a + random.gauss(0, distance_noise)   # initial x position
        self.y = b + random.gauss(0, distance_noise) # initial y position
        self.orientation = ang + random.gauss(0,angle_noise)  # initial orientation

    # --------
    # set:
    #    sets a robot coordinate
    #


    def set(self, x_new, y_new, orientation_new):
        """Set particle position on the field"""
        if -100 <= x_new <= WORLD_X+100:
            self.x = x_new
        else:
            raise Exception("Invalid x cord={}".format(x_new))
        if -100 <= y_new <= WORLD_Y+100:
            self.y = y_new
        else:
            raise Exception("Invalid y cord={}".format(y_new))
        self.orientation = orientation_new % (2 * math.pi)  # maybe need to add warning!


    def move(self, delta):
        """Move particle by creating new one and setting position"""
        x_new = self.x + delta[0] + random.gauss(0, distance_noise)
        y_new = self.y + delta[1] + random.gauss(0, distance_noise)
        orientation_new = self.orientation + delta[2] + random.gauss(0, angle_noise)
        new_robot = Robot()
        new_robot.set(x_new, y_new, orientation_new)
        return new_robot

    def pose(self):
        """Return particle pose"""
        return self.x, self.y, self.orientation
#ICP
    def gaussian(self, mu, sigma, x):
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return math.exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / math.sqrt(2.0 * math.pi * (sigma ** 2))

    def weight(self, lidar_data):
        """Calculate particle weight based on its pose and lidar data"""

        if len(lidar_data) == 1:
            minimum = INT_MAX
            best = -10
            for mark in landmarks:
                prob = 1.0
                dist = math.sqrt((self.x - mark[0]) ** 2 + (self.y - mark[1]) ** 2)
                prob *= self.gaussian(dist, sense_noise, lidar_data[0][1])
                if best < prob:
                    best = prob
            return best

        elif len(lidar_data) == 2:
            comb = [[0, 1], [0, 2], [1, 2], [1, 0], [2, 0], [2, 1]]  # Precalculated sets
            best = -10
            for st in comb:
                prob = 1.0
                dist = math.sqrt((self.x - landmarks[st[0]][0]) ** 2 + (self.y - landmarks[st[0]][1]) ** 2)
                prob *= self.gaussian(dist, sense_noise, lidar_data[0][1])# lidar_data (angle,dist)

                dist = math.sqrt((self.x - landmarks[st[1]][0]) ** 2 + (self.y - landmarks[st[1]][1]) ** 2)
                prob *= self.gaussian(dist, sense_noise, lidar_data[1][1])
                if best < prob:
                    best = prob
            return best

        elif len(lidar_data) == 3:
            comb = [[2, 0, 1], [1, 0, 2], [0, 1, 2], [2, 1, 0], [1, 2, 0], [0, 2, 1]]  # Precalculated sets
            best = -10
            for st in comb:
                prob = 1.0
                dist = math.sqrt((self.x - landmarks[st[0]][0]) ** 2 + (self.y - landmarks[st[0]][1]) ** 2)
                prob *= self.gaussian(dist, sense_noise, lidar_data[0][1])

                dist = math.sqrt((self.x - landmarks[st[1]][0]) ** 2 + (self.y - landmarks[st[1]][1]) ** 2)
                prob *= self.gaussian(dist, sense_noise, lidar_data[1][1])

                dist = math.sqrt((self.x - landmarks[st[2]][0]) ** 2 + (self.y - landmarks[st[2]][1]) ** 2)
                prob *= self.gaussian(dist, sense_noise, lidar_data[2][1])

                if best < prob:
                    best = prob
            return best
                #######
                # theory_dist = abs(lidar_data[0] - math.sqrt((self.x - landmarks[st[0]][0]) ** 2 + (
                #     self.y - landmarks[st[0]][1]) ** 2)) ** 2  # distance to first mark
                #
                # theory_dist += abs(lidar_data[1] - math.sqrt((self.x - landmarks[st[1]][0]) ** 2 + (
                #     self.y - landmarks[st[1]][1]) ** 2)) ** 2  # distance to second mark
                #
                # theory_dist += abs(lidar_data[2] - math.sqrt((self.x - landmarks[st[2]][0]) ** 2 + (
                #     self.y - landmarks[st[2]][1]) ** 2)) ** 2  # distance to third mark

                # theory_dist = dif1^2 + dif2^2 +dif3^2
        else:
            logging.warning("Invalid lidar data len={0}".format(len(lidar_data)))
            return 0.1



    def weight2(self, x_rob, y_rob, BEACONS):
        """Calculate particel weight based on its pose and lidar data"""
        temp_beac = [(beacon[0] - self.x, beacon[1] - self.y) for beacon in BEACONS]
        beacons = [(math.cos(self.orientation) * beac[0] + math.sin(self.orientation) * beac[1],
                    -math.sin(self.orientation) * beac[0] + math.cos(self.orientation) * beac[1])
                   for beac in temp_beac]



        beacon = [0, 0, 0]
        num_point = [0, 0, 0]
        for j in xrange(len(x_rob)):
            l1 = abs(math.sqrt((beacons[0][0] - x_rob[j]) ** 2 +
                               (beacons[0][1] - y_rob[j]) ** 2) - 40)
            l2 = abs(math.sqrt((beacons[1][0] - x_rob[j]) ** 2 +
                               (beacons[1][1] - y_rob[j]) ** 2) - 40)
            l3 = abs(math.sqrt((beacons[2][0] - x_rob[j]) ** 2 +
                               (beacons[2][1] - y_rob[j]) ** 2) - 40)
            lmin = l1
            num = 0
            if l2 < lmin:
                lmin = l2
                num = 1
            if l3 < lmin:
                lmin = l3
                num = 2
            if lmin > 200:
                continue
            beacon[num] += lmin
            num_point[num] += 1
        median = [(beacon[i] / num_point[i]) if num_point[i] != 0 else (1000) for i in xrange(3)]
        try:
            return MarkGaus(float(sum(median))/len(median))
        except ZeroDivisionError:
            print 'Zero division error in weights'
            return 0



    def __str__(self):
        """Print statement"""
        return 'Particle pose: x = %.2f mm, y = %.2f mm, theta = %.2f deg' \
               % (self.x, self.y, math.degrees(self.orientation))


def get_beacons(scan):
    #print scan
    scan = np.array(scan,dtype=float)
    #max_intens = scan[:, 1].max() * 0.8
    max_intens = 2400
    #print 'max'
    #print max_intens
    beacons = []
    prev = -1
    dist = 99999
    for i in range(1080):
        if scan[i][1] > max_intens:
            # print scan[i][1]
            if prev == -1:
                dist = scan[i][0]
                prev = i
                continue
            if scan[i][0] < dist and (abs(dist - scan[i][0]) < 100) and abs(i - prev) < 30:
                dist = scan[i][0]
                prev = i
            elif abs(dist - int(scan[i][0])) > 100 or abs(i - prev) > 30:
                beacons.append((prev * 0.25 - 135, dist))
                prev = i
                dist = scan[i][0]
    if len(beacons)==0 and dist!=99999:
        beacons.append((prev * 0.25 - 135, dist))
    elif abs(beacons[-1][1]-dist)>100 or (beacons[-1][0]+135)/0.25-prev>30:
        # print prev * 0.25 - 135
        beacons.append((prev * 0.25 - 135, dist))
    logging.info('beacons'+','.join([str(i[1])+'-'+str(i[0])for i in beacons]))
    return beacons


def get_beacons_Marko(scan):
    angles = []
    distances = []
    max_intens = 2800 #2800
    for i in range(len(scan)):
        if scan[i][1]>max_intens and scan[i][0]<3700:
            angles.append(i*math.pi/180/4)
            distances.append(scan[i][0])
    return angles,distances



def get_beacons_2(scan):
    scan = np.array(scan,dtype=float)
    max_intens = 2800
    beacons = []
    new_scan = []
    for i in range(len(scan)):
        if(scan[i][1]>max_intens):
            new_scan.append((i,scan[i][0]))
    if len(new_scan)==0:
        return new_scan

    beacons.append(new_scan[0])
    for i in new_scan:
        if i[1] < beacons[-1][1] and abs(i[1]-beacons[-1][1]) < 150 and abs(i[0]-beacons[-1][0]) < 30:
            beacons.pop()
            beacons.append(i)

        elif (i[1]-beacons[-1][1]) >= 150 or abs(i[0]-beacons[-1][0]) >= 30:
            beacons.append(i)
    answer = []
    for i in beacons:
        if(i[1]<3900):
            answer.append(i)
    return answer


def angle5(angle):
    """Transform lidar points from lidar coord sys to robot cord sys"""
    # if angle >= math.pi/4:
    #	return angle - math.pi/4
    # else:
    #	return angle + 7*math.pi/4
    return (angle + math.pi / 4) % (2 * math.pi)


def p_trans(agl, pit):
    """Transform lidar measurment to xy in robot coord sys"""
    x_rob = [-1*pit[i] * math.cos(angle5(agl[i])) for i in xrange(len(agl))]
    y_rob = [-1*pit[i] * math.sin(angle5(agl[i])) for i in xrange(len(agl))]
    return x_rob, y_rob




#def  get_beacons(scan):





def resample(p, w, N):
    """Random pick algorithm for resempling"""
    sample = []
    index = int(random.random() * N)
    beta = 0.0
    mw = max(w)
    for i in xrange(N):
        beta += random.random() * 2.0 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % N
        sample.append(p[index])
    return sample


def calculate_main(particles):
    """Function returns robot with mean data"""
    p_x = 0
    p_y = 0
    p_orient = 0
    for i in range(len(particles)):
        p_x += particles[i].x
        p_y += particles[i].y
        p_orient += (((particles[i].orientation - particles[0].orientation + math.pi) % (2.0 * math.pi))
                        + particles[0].orientation - math.pi)

    answer = Robot()
    answer.set(p_x / len(particles), p_y / len(particles), p_orient / len(particles))
    logging.info(answer)
    return answer


def particles_move(particles, delta):
    """Function returns new particle set after movement"""
    for i in range(len(particles)):
        particles[i] = particles[i].move(delta)
    return particles


def particles_sense(particles, lidar_data):
    """Function returns new particle set after sensing"""
    weights = []
    for i in range(len(particles)):
        weights.append(particles[i].weight(lidar_data))
    sm = sum(weights)
    if sm==0:
        weights = [0.0 for i in xrange(len(particles))]
        answer = resample(particles, weights, particle_number)
        return answer
    for i in range(len(weights)):
        weights[i] = weights[i] / sm
        #print i
        #print particles[i]
        #print weights[i]
        #print '##########'
    answer = resample(particles, weights, particle_number)
    return answer

def particle_sense2(particles,scan):
    plt.figure(0)
    angle,distance = get_beacons_Marko(scan)
    x_coords, y_coords = p_trans(angle, distance)
    print x_coords
    print y_coords
    mmmain = calculate_main(particles)
    #plt.plot([i for i in x_coords], [i for i in y_coords], 'ro',color = 'r')
    #for i in landmarks:
    #    plt.plot(i[0],i[1],'ro',markersize=10,color = 'b')
    weights = []
    #plt.show()
    for i in range(len(particles)):
        weights.append(particles[i].weight2(x_coords,y_coords,landmarks))
    sm = sum(weights)
    if sm==0:
        weights = [0.0 for i in xrange(len(particles))]
        answer = resample(particles, weights, particle_number)
        return answer
    for i in range(len(weights)):
        weights[i] = weights[i] / sm
    answer = resample(particles, weights, particle_number)
    #for i in range(len(particles)):
        #print i
        #print particles[i]
        #print weights[i]
        #print '##########'
        #plt.plot(particles[i].x,particles[i].y,'ro',markersize=weights[i]*10,color = 'g')
    #plt.show()
    return answer



def localisation():
    # initialize particles
    particles = [Robot() for i in range(particle_number/2)]
    for i in range(particle_number/2):
        t = Robot()
        t.set(50 ,50 , 0)
        particles.append(t)
    # initialize main robot
    #main_robot = calculate_main(particles)
    #print(main_robot)

    delta = [15, 15, 0]
    # move ->sense
    particles = particles_move(particles, delta)
    main_robot = calculate_main(particles)
    print(main_robot)
    lidar_data = [35.355339, 106.066017, 79.05694]

    for i in range(10):
        particles = particles_sense(particles, lidar_data)
        main_robot = calculate_main(particles)
        print main_robot
    #particles = particles_sense(particles, lidar_data)
    #main_robot = calculate_main(particles)
    #print main_robot
    #particles = particles_sense(particles, lidar_data)
    #main_robot = calculate_main(particles)
    #print main_robot


#######
# test data
#######

#test_lidar = []

#print("finish")
#rob = Robot()
#rob.set(0, 0, 0)
#lid = [35.355339, 106.066017, 79.05694]
#lid = [50, 141.4213562373, 100]
#print(rob.weight(lid))
#rob.set(24, 24, 0)
#print(rob.weight(lid))
#rob.set(23, 25, 0)
#print(rob.weight(lid))
#localisation()


def pf_test():
    #  test
    rob = Robot()
    particles = [Robot() for i in range(particle_number)]
    delta = [30,0,0] # 220,170,0
    particles = particles_move(particles, delta)
    #print ideal(170, 170)# 870,2827,3382
    #[852,3415]
    main_robot = calculate_main(particles)
    print main_robot
    for i in range(30):
        particles = particles_sense(particles, ideal(198, 170)) # 3439, 836
        main_robot = calculate_main(particles)
        print main_robot

#pf_test()