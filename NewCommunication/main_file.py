import driver
import time
from hokuyolx import HokuyoLX
import logging
import signal
import npParticle as pf
import numpy as np
from multiprocessing import Process, Queue, Value,Array
from multiprocessing.queues import Queue as QueueType
lvl = logging.INFO
logging.basicConfig(filename='Eurobot.log', filemode='w', format='%(levelname)s:%(asctime)s %(message)s',
                    datefmt='%m/%d/%Y %I:%M:%S %p', level=lvl)

console = logging.StreamHandler()
console.setLevel(lvl)
# set a format which is simpler for console use
formatter = logging.Formatter('%(levelname)s: %(message)s')
console.setFormatter(formatter)
# add the handler to the root logger
logging.getLogger('').addHandler(console)
logger = logging.getLogger(__name__)


class Robot:
    def __init__(self, lidar_on=True):
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
        #self.x = 170  # mm
        #self.y = 150  # mm
        #self.angle = 0.0  # pi
        self.coords = Array('d',[170, 150, 0])
        self.input_queue = Queue()
        self.loc_queue = Queue()
        self.fsm_queue = Queue()
        self.PF = pf.ParticleFilter(particles=500, sense_noise=20, distance_noise=20, angle_noise=0.2, in_x=self.coords[0],
                                    in_y=self.coords[1],input_queue=self.input_queue,out_queue=self.loc_queue)


        # driver process
        self.dr = driver.Driver(self.input_queue,self.fsm_queue,self.loc_queue)
        p = Process(target=self.dr.run)
        p.start()
        p2 = Process(target=self.PF.localisation,args=(self.coords,self.get_raw_lidar))
        p2.start()
        # Test command
        #self.input_queue.put({'source':'fsm','cmd':'echo','params':'ECHO'})
        logging.info(self.send_command('echo','ECHO'))



        #logging.info(self.dr.process_cmd('echo', 'ECHO'))
        # Set start coordinates
        #self.input_queue.put({'source': 'fsm', 'cmd': 'echo', 'params': 'ECHO'})
        #logging.info(self.dr.process_cmd('setCoordinates',[self.x / 1000., self.y / 1000., self.angle]))
        logging.info(self.send_command('setCoordinates',[self.coords[0] / 1000., self.coords[1] / 1000., self.coords[2]]))


    def send_command(self,name,params=None):
        self.input_queue.put({'source':'fsm','cmd':name,'params':params})
        return self.fsm_queue.get()

    def get_raw_lidar(self):
        # return np.load('scan.npy')[::-1]
        timestamp, scan = self.lidar.get_intens()
        return scan
        # return scan[::-1]  our robot(old)

    def check_lidar(self):
        try:
            state = self.lidar.laser_state()
        except:
            self.lidar_on = False
            logging.warning('Lidar off')

    def go_to_coord_rotation(self, parameters):  # parameters [x,y,angle,speed]
        pm = [self.coords[0]/1000.,self.coords[1]/1000.,float(self.coords[2]),parameters[0] / 1000., parameters[1] / 1000., float(parameters[2]), parameters[3]]
        logging.info(self.dr.process_cmd('go_to_with_corrections',pm))
        # After movement
        stamp = time.time()
        time.sleep(0.100001)  # sleep because of STM interruptions (Maybe add force interrupt in STM)
        while not self.dr.process_cmd('is_point_was_reached')['data']:
            time.sleep(0.05)
            # add Collision Avoidance there
            if (time.time() - stamp) > 30:
                return False  # Error, need to handle somehow (Localize and add new point maybe)
        logging.info('point reached')
        #self.PF.move_particles([parameters[0] - self.coords[0], parameters[1] - self.coords[1], parameters[2] - self.coords[2]])
        #self.coords[0] = parameters[0]
        #self.coords[1] = parameters[1]
        #self.coords[2] = parameters[2]
        #print 'Before Calculation:'
        #self.PF.calculate_main()
        #if self.lidar_on:
        #    self.check_lidar()  # check lidar connection
        #if self.lidar_on:
        #    lidar_data = self.get_raw_lidar()
            # print lidar_data
            # TODO http://hokuyolx.readthedocs.io/en/latest/ RESET,REBOOT functions
        #    self.PF.particle_sense(lidar_data)
        #    print 'After Calculation:'
        #    main_robot = self.PF.calculate_main()
        #    self.coords[0] = main_robot[0]
        #    self.coords[1] = main_robot[1]
        #    self.coords[2] = main_robot[2]

        return True

    def go_last(self,parameters):
        while abs(parameters[0]-self.coords[0]) > 10 or abs(parameters[1]-self.coords[1]) > 10:
            print 'calibrate'
            self.go_to_coord_rotation(parameters)


    ############################################################################
    ######## HIGH LEVEL FUNCTIONS ##############################################
    ############################################################################
    def demo(self, speed=1):
        """robot Demo, go to coord and take cylinder"""
        signal.signal(signal.SIGALRM, self.funny_action)
        signal.alarm(90)
        # TODO take cylinder
        angle = np.pi/2.
        parameters = [850, 150, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1000, 500, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1000, 700, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [650, 1350, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [250, 1350, angle, speed]
        self.go_to_coord_rotation(parameters)
        self.go_last(parameters)

    def demo_r(self, speed=1):
        """robot Demo, go to coord and take cylinder"""
        # TODO take cylinder
        angle = np.pi / 2.
        parameters = [650, 1350, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1000, 700, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [1000, 500, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [850, 150, angle, speed]
        self.go_to_coord_rotation(parameters)
        parameters = [170, 150, angle, speed]
        self.go_to_coord_rotation(parameters)

    def funny_action(self, signum, frame):
        print 'Main functionaly is off'
        print 'FUNNNY ACTION'


def test():
    rb = Robot(True)
    return
    i = 0
    while i<10:
        rb.demo(4)
        rb.demo_r(4)
        i+=1

def tst_time():
    rb = Robot(True)
    stamp = time.time()
    for i in range(100):
        command = {'source': 'fsm', 'cmd': 'echo', 'params': 'ECHO'}
        rb.dr.process_cmd(command)
    print time.time()-stamp

test()

