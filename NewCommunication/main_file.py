import driver
import time
from hokuyolx import HokuyoLX
import logging
import signal
import npParticle as pf

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


class Robot:
    def __init__(self,lidar_on=True):
        self.lidar_on = lidar_on
        if lidar_on:
            logging.debug('lidar is connected')
            # add check for lidar connection
            try:
                self.lidar = HokuyoLX(tsync=False)
                self.lidar.convert_time = False
            except:
                self.lidar_on = False
                logging.warning('lidar is not connected')
        self.x = 170  # mm
        self.y = 150  # mm
        self.angle = 0.0  # pi
        self.PF = pf.ParticleFilter(particles=500,sense_noise=50,distance_noise=30,angle_noise=0.02,in_x = self.x,in_y = self.y)
        self.dr = driver.Driver(1, 2, 3)
        self.dr.connect()
        # Test command
        command = {'source': 'fsm', 'cmd': 'echo', 'params': 'ECHO'}
        logging.info(self.dr.process_cmd(command))
        # Set start coordinates
        command = {'source': 'fsm', 'cmd': 'setCoordinates', 'params': [self.x/1000., self.y/1000., self.angle]}
        logging.info(self.dr.process_cmd(command))

    def get_raw_lidar(self):
        # return np.load('scan.npy')[::-1]
        timestamp, scan = self.lidar.get_intens()
        return scan
        # return scan[::-1]  our robot(old)

    def go_to_coord_rotation(self,parameters): #  parameters [x,y,angle,speed]
        pm = [parameters[0] / 1000., parameters[1] / 1000., parameters[2], parameters[3]]
        command = {'source': 'fsm', 'cmd': 'addPointToStack', 'params': pm}
        logging.info(self.dr.process_cmd(command))
        # After movement
        stamp = time.time()
        cmd = {'source': 'fsm', 'cmd': 'is_point_was_reached'}
        time.sleep(0.100001) # sleep because of STM interruptions (Maybe add force interrupt in STM)
        while not self.dr.process_cmd(cmd)['data']:
            time.sleep(0.05)
            if (time.time() - stamp) > 30:
                return False # Error, need to handle somehow (Localize and add new point maybe)
        logging.info('point reached')
        self.PF.move_particles([parameters[0] - self.x, parameters[1] - self.y, parameters[2] - self.angle])
        self.x = parameters[0]
        self.y = parameters[1]
        self.angle = parameters[2]
        print 'Before Calculation:'
        self.PF.calculate_main()
        if self.lidar_on:
            lidar_data = self.get_raw_lidar() # check lidar connection!
            #print lidar_data
            # TODO http://hokuyolx.readthedocs.io/en/latest/ RESET,REBOOT functions
            self.PF.particle_sense(lidar_data)
            print 'After Calculation:'
            main_robot = self.PF.calculate_main()
            self.x = main_robot[0]
            self.y = main_robot[1]
            self.angle = main_robot[2]
        # else lidar
        command = {'source': 'fsm', 'cmd': 'setCoordinates', 'params': [self.x / 1000., self.y / 1000., self.angle]}
        logging.info(self.dr.process_cmd(command))
        # TODO add move correction

    ############################################################################
    ######## HIGH LEVEL FUNCTIONS ##############################################
    ############################################################################
    def demo(self,speed= 1):
        """robot Demo, go to coord and take cylinder"""
        signal.signal(signal.SIGALRM, self.funny_action)
        signal.alarm(90)
        # TODO take cylinder
        parameters = [850, 150, 0.0, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1000, 500, 0.0, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1000, 700, 0.0, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [650, 1350, 0.0, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [250, 1350, 0.0, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [250, 1350, 0.0, speed]
        self.go_to_coord_rotation(parameters)

    def demo_r(self,speed= 1):
        """robot Demo, go to coord and take cylinder"""
        # TODO take cylinder
        parameters = [850, 150, 0.0, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1000, 500, 0.0, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1000, 700, 0.0, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [650, 1350, 0.0, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [250, 1350, 0.0, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [250, 1350, 0.0, speed]
        self.go_to_coord_rotation(parameters)

    def funny_action(self,signum, frame):
        print 'Main functionaly is off'
        print 'FUNNNY ACTION'



def test():
    rb = Robot(True)
    rb.demo()

test()




