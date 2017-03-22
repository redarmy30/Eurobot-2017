#!/usr/bin/env python

import sys
import time
import json
from multiprocessing import Process, Queue, Value,Array

from hokuyolx import HokuyoLX
#import main_file

import smach
#import smach_ros
import rospy

import logging
logging.getLogger('rosout')
# class Filter(logging.Filter):
#     def filter(self, record):
#         return 'State machine transitioning' not in record.msg
# logging.getLogger('rosout').addFilter(Filter())

class Robot:
    def __init__(self, lidar_on=True):
        self.lidar_on = lidar_on
        # if lidar_on:
        #     logging.debug('lidar is connected')
        #     # add check for lidar connection
        #     try:
        #         self.lidar = HokuyoLX(tsync=False)
        #         self.lidar.convert_time = False
        #     except:
        #         self.lidar_on = False
        #         logging.warning('lidar is not connected')
        # #self.x = 170  # mm
        # #self.y = 150  # mm
        # #self.angle = 0.0  # pi
        # self.coords = Array('d',[170, 150, 0])
        # self.input_queue = Queue()
        # self.loc_queue = Queue()
        # self.fsm_queue = Queue()
        # self.PF = pf.ParticleFilter(particles=800, sense_noise=30, distance_noise=30, angle_noise=0.1, in_x=self.coords[0],
        #                             in_y=self.coords[1],input_queue=self.input_queue,out_queue=self.loc_queue)
        #
        #
        # # driver process
        # self.dr = driver.Driver(self.input_queue,self.fsm_queue,self.loc_queue)
        # p = Process(target=self.dr.run)
        # p.start()
        # p2 = Process(target=self.PF.localisation,args=(self.coords,self.get_raw_lidar))
        # logging.info(self.send_command('echo','ECHO'))
        # logging.info(self.send_command('setCoordinates',[self.coords[0] / 1000., self.coords[1] / 1000., self.coords[2]]))
        # p2.start()

    def send_command(self,name,params=None):
        pass

    def get_raw_lidar(self):
        pass

    def check_lidar(self):
        pass

    def go_to_coord_rotation(self, parameters):  # parameters [x,y,angle,speed]
        pass

    def go_last(self,parameters):
        pass
def parse_strategy(strategy_text):

    def isfloat(value):
        try:
            float(value)
            return True
        except:
            return False

    res = [[part.lstrip().rstrip() for part in line.split(":")]
                for line in strategy_text.split('\n') if line[0] != '#']

    #for r in res

class Timer(smach.State):
    def __init__(sf, outcomes=[], input_keys=[], output_keys=[]):
        smach.State.__init__(sf, outcomes, input_keys, output_keys)

    def execute(sf, ud):
        while 'start_time' not in ud:
            rospy.loginfo('>>> Waiting for data...')
            rospy.sleep(0.5)
        rospy.loginfo('>>> GOT DATA! x = '+str(ud.start_time))
        return '90 seconds'

class RobotInit(smach.State):
    def __init__(sf, outcomes=[], input_keys=[], output_keys=[]):
        smach.State.__init__(sf, outcomes, input_keys, output_keys)

    def execute(sf, ud):
        time.sleep(3)
        ud.start_time = time.time()
        return 'robot initialized'

class CommandHandler(smach.State):
    def __init__(sf, outcomes=[], input_keys=[], output_keys=[]):
        smach.State.__init__(sf, outcomes, input_keys, output_keys)

    def execute(sf, ud):
        return 'strategy ended'

class Move(smach.State):
    def __init__(sf, outcomes=[], input_keys=[], output_keys=[]):
        smach.State.__init__(sf, outcomes, input_keys, output_keys)

    def execute(sf, ud):
        return 'succeeded'

class TakeModule(smach.State):
    def __init__(sf, outcomes=[], input_keys=[], output_keys=[]):
        smach.State.__init__(sf, outcomes, input_keys, output_keys)

    def execute(sf, ud):
        return 'succeeded'

class PutModule(smach.State):
    def __init__(sf, outcomes=[], input_keys=[], output_keys=[]):
        smach.State.__init__(sf, outcomes, input_keys, output_keys)

    def execute(sf, ud):
        return 'succeeded'

def start_fsm():
    with open(sys.argv[1]) as config_f:
        # def ascii_encode_dict(data):
        #     ascii_encode = lambda x: x.encode('ascii')
        #     return dict(map(ascii_encode, pair) for pair in data.items())

        config_dict = json.load(config_f)
        # print type(config_dict['robot name'])
        # for x in config_dict:
        #     print x,':',config_dict[x]

    # with open(sys.argv[2]) as f_strategy_f:
    #     strategy_txt = json.load(f_strategy_f)

    startup_fsm = smach.Concurrence(
        outcomes=['timer end', 'succeeded strategy', 'aborted'],
        default_outcome='timer end',
        input_keys=[],
        output_keys=[],
        #TODO#        child_termination_cb = child_term_cb, // use instead of outcome_map
        #TODO#        outcome_cb = out_cb,
        outcome_map = {
            'succeeded strategy': {'ROBOT': 'strategy ended'},
            'timer end': {'TIMER': '90 seconds'},
            'aborted': {'ROBOT': 'aborted'}
        }
    )
    startup_fsm.userdata.robot_data = config_dict['robot data']
    actions =[action_str.encode('ascii','ignore') for action_str in config_dict['strategy actions']]
    startup_fsm.userdata.game_field = config_dict['game field']

    with open(sys.argv[2]) as f_rob_strategy:
        f_rob_strategy_text = f_rob_strategy.read()
        startup_fsm.userdata.parsed_strategy = parse_strategy(f_rob_strategy_text)

    print(startup_fsm.userdata.parsed_strategy)

    #print actions + ['strategy ended']
    #print {action:action.upper() for action in actions}.update(
    #    {'strategy ended':'strategy ended'})

    with startup_fsm:
        smach.Concurrence.add('TIMER', Timer(outcomes=['90 seconds'],
                                             input_keys=['start_time'],
                                             output_keys=['deadline1']))

        sm_robot = smach.StateMachine(outcomes=['aborted', 'strategy ended'],
                                      input_keys = ['robot_data', 'game_field', 'parsed_strategy'],
                                      output_keys=['rb', 'start_time'])
        sm_robot.userdata.rb = Robot()
        with sm_robot:
            smach.StateMachine.add('ROBOT INIT', RobotInit(outcomes=['robot initialized', 'initialization failed'],
                                                           input_keys=['robot_data', 'parsed_strategy'],
                                                           output_keys=['strategy_list', 'start_time']),
                                   transitions={'robot initialized': 'COMMAND HANDLER',
                                                'initialization failed': 'aborted'})

            command_handler_trans = {action:action.upper() for action in actions}
            command_handler_trans.update({'strategy ended':'strategy ended'})

            smach.StateMachine.add('COMMAND HANDLER', CommandHandler(outcomes=actions + ['strategy ended'],
                                                                     input_keys=['strategy_list'],
                                                                     output_keys=actions),
                                   transitions=command_handler_trans)


            smach.StateMachine.add('TAKE MODULE', TakeModule())
            smach.StateMachine.add('PUT MODULE', PutModule())

            motion_fsm = smach.Concurrence(
                outcomes=['succeeded', 'collision'],
                default_outcome='succeeded',
                input_keys=['parameters'],
                output_keys=['new_point'],
                #TODO#        child_termination_cb = child_term_cb, // use instead of outcome_map
                #TODO#        outcome_cb = out_cb,
                # outcome_map = {
                #     'succeded':{f_rob_name.upper():'succeded', s_rob_name.upper():'succeded'},
                #     'aborted ' + f_rob_name: {f_rob_name.upper():'aborted',s_rob_name.upper():'succeded'},
                #     'aborted ' + s_rob_name: {f_rowb_name.upper():'succeded',s_rob_name.upper():'aborted'},
                #     'aborted both': {f_rob_name.upper():'aborted',s_rob_name.upper():'aborted'}
                # }a
            )
            smach.StateMachine.add('MOVE', motion_fsm,
                                   transitions={'succeeded': 'COMMAND HANDLER',
                                                'collision': 'COMMAND HANDLER'})

        smach.Concurrence.add('ROBOT', sm_robot)

        startup_fsm.execute()

if __name__ == "__main__":
    start_fsm()
