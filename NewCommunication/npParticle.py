
import numpy as np
import time
import logging
import math

# Dimensions of the playing field
WORLD_X = 3000
WORLD_Y = 2000
mark_r = 40
landmarks = np.array([[-mark_r, WORLD_Y / 2.], [WORLD_X + mark_r, WORLD_Y + mark_r], [WORLD_X + mark_r, - mark_r]])


class ParticleFilter:
    def __init__(self,particles=500,sense_noise=50,distance_noise=30,angle_noise=0.02,in_x = 150,in_y = 150):
        stamp = time.time()
        self.particles_num = particles
        self.sense_noise = sense_noise
        self.distance_noise = distance_noise
        self.angle_noise = angle_noise
        x = np.random.normal(in_x, distance_noise, particles)
        y = np.random.normal(in_y, distance_noise, particles)
        orient = np.random.normal(0, angle_noise, particles) % (2 * math.pi)
        self.particles = np.vstack((x,y,orient)).T
        logging.info('initialize time: '+str(time.time()-stamp))

    def gaus(self, x):
        """calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma"""
        return np.exp(- (x ** 2) / (self.sense_noise ** 2) / 2.0) / np.sqrt(2.0 * np.pi * (self.sense_noise ** 2))

    def move_particles(self, delta): # delta = [dx,dy,d_rot]
        stamp = time.time()
        # # self.particles + noise + delta:
        # # noise - Nx3 : N - num particles, (x_noise, y_noise, angle_noise)
        # self.particles += (np.random.normal(loc=np.array([0, 0, 0]),
        #                                     scale=np.diag([self.distance_noise, self.distance_noise, self.angle_noise]),
        #                                     size=(self.particle_num, 3))
        #                    + np.array([delta]))
        # self.particles[:, 2] %= 2 * np.pi

        x_noise = np.random.normal(0, self.distance_noise, self.particles_num)
        y_noise = np.random.normal(0, self.distance_noise, self.particles_num)
        angle_noise = np.random.normal(0, self.angle_noise, self.particles_num)
        self.particles[:, 0] = self.particles[:, 0] + delta[0] + x_noise
        self.particles[:, 1] = self.particles[:, 1] + delta[1] + y_noise
        self.particles[:, 2] = (self.particles[:, 2] + delta[2] + angle_noise) % (2 * math.pi)
        logging.info('Particle Move time: ' + str(time.time() - stamp))

    def resample(self, weights):
        n = self.particles_num
        indices = []
        C = [0.] + [sum(weights[:i + 1]) for i in range(n)]
        u0, j = np.random.rand(), 0
        for u in [(u0 + i) / n for i in range(n)]:
            while u > C[j]:
                j += 1
            indices.append(j - 1)
        return indices

    def calculate_main(self):
        stamp = time.time()
        x = np.mean(self.particles[:, 0])
        y = np.mean(self.particles[:, 1])
        zero_elem = self.particles[:, 2][0]
        temporary = ((self.particles[:, 2]-zero_elem+np.pi) % (2.0 * np.pi))+zero_elem-np.pi
        orient = np.mean(temporary)
        answer = x, y, orient
        logging.info('main_calculation time' + str(time.time() - stamp))
        logging.info(answer)
        return answer

    def particle_sense(self, scan):
        stamp = time.time()
        angle,distance = get_landmarks(scan)
        x_coords,y_coords = p_trans(angle,distance)
        weights = self.weights(x_coords,y_coords,landmarks)
        self.particles = self.particles[self.resample(weights), :]
        logging.info('particle_sense time :' + str(time.time() - stamp)+" points: "+str(len(x_coords)))
        return self.particles

    def weights(self, x_rob, y_rob, BEACONS):
        """Calculate particle weight based on its pose and lidar data"""
        # TODO check ICP implementation
        res = BEACONS[np.newaxis, :, :] - self.particles[:, np.newaxis, :2]
        X = res[:,:,0]*np.cos(self.particles[:,2])[:, np.newaxis] + res[:,:,1]*np.sin(self.particles[:,2])[:, np.newaxis]
        Y = -res[:, :, 0] * np.sin(self.particles[:, 2])[:, np.newaxis] + res[:, :, 1] * np.cos(self.particles[:, 2])[:, np.newaxis]
        beacon = np.concatenate((X[:, :, np.newaxis], Y[:, :, np.newaxis]), axis=2)
        # beacon = beacons are in local coordinates
        ln1 = np.abs(np.sqrt((beacon[:, np.newaxis, 0, 0] - x_rob[np.newaxis, :])**2 + (beacon[:, np.newaxis, 0, 1] - y_rob[np.newaxis, :])**2) - 40)
        ln2 = np.abs(np.sqrt((beacon[:, np.newaxis, 1, 0] - x_rob[np.newaxis, :])**2 + (beacon[:, np.newaxis, 1, 1] - y_rob[np.newaxis, :])**2) - 40)
        ln3 = np.abs(np.sqrt((beacon[:, np.newaxis, 2, 0] - x_rob[np.newaxis, :])**2 + (beacon[:, np.newaxis, 2, 1] - y_rob[np.newaxis, :])**2) - 40)

        # lns are differences in theoretical and real data
        # ln1,ln2,ln3 are correct
        beacon_error_sum = np.empty([self.particles_num,3],dtype=np.float)
        errors = np.minimum(ln1, np.minimum(ln2, ln3))

        threshold = 200
        limit_err = errors > threshold
        error_l1 = np.logical_and(np.equal(errors, ln1), ~limit_err)
        error_l2 = np.logical_and(np.equal(errors, ln2), ~limit_err)
        error_l3 = np.logical_and(np.equal(errors, ln3), ~limit_err)
        err_l1 = np.sum(error_l1, axis=-1)
        err_l2 = np.sum(error_l2, axis=-1)
        err_l3 = np.sum(error_l3, axis=-1)
        beacon_error_sum[...] = 1000
        ind = np.where(err_l1)[0]
        if ind.size:
            beacon_error_sum[ind, 0] = np.sum(np.where(error_l1, errors, 0), axis=-1)[ind] / err_l1[ind]
        ind = np.where(err_l2)[0]
        if ind.size:
            beacon_error_sum[ind, 1] = np.sum(np.where(error_l2, errors, 0), axis=-1)[ind] / err_l2[ind]
        ind = np.where(err_l3)[0]
        if ind.size:
            beacon_error_sum[ind, 2] = np.sum(np.where(error_l3, errors, 0), axis=-1)[ind] / err_l3[ind]
        weights = self.gaus(np.mean(beacon_error_sum, axis=1))
        weights /= np.sum(weights)
        return weights
        # TODO try use median instead mean
        # TODO if odometry works very bad and weights are small use only lidar


# help functions

def get_landmarks(scan):
    """Returns filtrated lidar data"""
    stamp = time.time()
    max_intens = 2600
    max_dist = 3700
    ind = np.where(np.logical_and(scan[:, 1] > max_intens, scan[:, 0] < max_dist))[0]
    angles = np.pi / 4 / 180 * ind
    distances = scan[ind, 0]
    logging.info('scan preproccesing time: ' + str(time.time() - stamp))
    return (angles + np.pi / 4+ np.pi/2) % (2 * np.pi),distances # delete +np.pi for our robot


def p_trans(agl, pit):
    x_rob = pit*np.cos(agl) # multiply by minus in our robot
    y_rob = pit*np.sin(agl)
    return x_rob,y_rob










