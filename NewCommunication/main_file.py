import  cmd_list
import  driver
import  packets
import time
import ParticleFilter as pf
from hokuyolx import HokuyoLX
import matplotlib.pyplot as plt
import logging
import numpy as np



lvl = logging.INFO
logging.basicConfig(filename='Eurobot.log', filemode='w',format='%(levelname)s:%(asctime)s %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p',level=lvl)

console = logging.StreamHandler()
console.setLevel(lvl)
# set a format which is simpler for console use
formatter = logging.Formatter('%(levelname)s: %(message)s')
console.setFormatter(formatter)
# add the handler to the root logger
logging.getLogger('').addHandler(console)
logger = logging.getLogger(__name__)

class Robot():
    def __init__(self,lidar_on=True):
        if lidar_on:
            logging.debug('lidar is connected')
            # add check for lidar connection
            self.lidar = HokuyoLX(tsync=False)
            self.lidar.convert_time = False
        self.lidar_on = lidar_on
        self.x = 1000  # mm
        self.y = 1500  # mm
        self.angle = 0.0  # pi
        self.particles = [pf.Particle(self.x,self.y,self.angle) for i in range(pf.particle_number)]
        self.dr = driver.Driver(1, 2, 3)
        self.dr.connect()
        # Test command
        command = {'source': 'fsm', 'cmd': 'echo', 'params': 'ECHO'}
        logging.info(self.dr.process_cmd(command))
        # Set start coordinates
        command = {'source': 'fsm', 'cmd': 'setCoordinates', 'params': [self.x/1000., self.y/1000., self.angle]}
        logging.info(self.dr.process_cmd(command))



    def get_raw_lidar(self):
        #return np.load('scan.npy')[::-1]
        timestamp, scan = self.lidar.get_intens()
        return scan[::-1]


    def go_to_coord_rotation(self,parameters): #  parameters [x,y,angle,speed]
        pm = [parameters[0] / 1000., parameters[1] / 1000., parameters[2], parameters[3]]
        command = {'source': 'fsm', 'cmd': 'addPointToStack', 'params': pm}
        logging.info(self.dr.process_cmd(command))
        # After movement
        stamp = time.time()

        cmd = {'source': 'fsm', 'cmd': 'is_point_was_reached'}
        time.sleep(0.100001)
        while not self.dr.process_cmd(cmd)['data']:
            time.sleep(0.05)
            if (time.time() - stamp) > 30:
                return False # Error
        # After movement
        stamp = time.time()
        time.sleep(0.05)
        logging.info('point reached')
        self.particles = pf.particles_move(self.particles, [parameters[0]-self.x,parameters[1]-self.y,parameters[2]-self.angle])
        self.x = parameters[0]
        self.y = parameters[1]
        self.angle = parameters[2]
        print 'Before Calculation:'
        pf.calculate_main(self.particles)
        if self.lidar_on:
            lidar_data = self.get_raw_lidar()
            #print lidar_data
            self.particles = pf.particles_sense(self.particles,lidar_data)
            print 'After Calculation:'
            main_robot = pf.calculate_main(self.particles)
            self.x = main_robot.x
            self.y = main_robot.y
            self.angle = main_robot.orientation
        # else lidar
        command = {'source': 'fsm', 'cmd': 'setCoordinates', 'params': [self.x / 1000., self.y / 1000., self.angle]}
        logging.info(self.dr.process_cmd(command))
        print time.time()-stamp
        # TODO add move correction


    def demo(self):
        """robot Demo, go to coord and take cylinder"""
        # TODO take cylinder
        parameters = [650, 650, 0.0, 4]
        self.go_to_coord_rotation(parameters)
        parameters = [250, 650, 0.0, 4]
        self.go_to_coord_rotation(parameters)
        parameters = [250, 650, 0.0, 4]
        self.go_to_coord_rotation(parameters)
        parameters = [250, 650, 0.0, 4]
        self.go_to_coord_rotation(parameters)


def test():
    rb = Robot(True)
    rb.demo()

test()




