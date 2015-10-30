#! /usr/bin/env python
import rospy
import rospkg
import rosbag
import sys
import csv
import glob
import csv_fcns
from math import floor
from yaml import load

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

BOOL_TYPE = 'std_msgs/Bool'
TWIST_TYPE = 'geometry_msgs/Twist'
TWISTSTAMPED_TYPE = 'geometry_msgs/TwistStamped'
FIX_TYPE = 'sensor_msgs/NavSatFix'
IMU_TYPE = 'sensor_msgs/Imu'
THROTTLE_REPORT_TYPE = 'dbw_mkz_msgs/ThrottleReport'
BRAKE_REPORT_TYPE = 'dbw_mkz_msgs/BrakeReport'
STEERING_REPORT_TYPE = 'dbw_mkz_msgs/SteeringReport'
GEAR_REPORT_TYPE = 'dbw_mkz_msgs/GearReport'
FUEL_LEVEL_REPORT_TYPE = 'dbw_mkz_msgs/FuelLevelReport'
TIRE_PRESSURE_REPORT_TYPE = 'dbw_mkz_msgs/TirePressureReport'
WHEEL_SPEED_REPORT_TYPE = 'dbw_mkz_msgs/TirePressureReport'
MISC1_REPORT_TYPE = 'dbw_mkz_msgs/Misc1Report'

bag_topics = []
csv_topics = []
csv_types = []

def get_csv_type(topic_name):
    global csv_topics
    
    for i in xrange(0, len(csv_topics)):
        if csv_topics[i] == topic_name:
            return i
        
    return -1    

def bag_to_csv():
    global csv_writer
    global bag_topics
    global csv_topics
    global csv_types
    
    bags = glob.glob('*.bag')
    print 'Bag files found: ' + str(len(bags))
    for fname in bags:
        print '    ' + fname
    
    yaml_file = open('params.yaml', 'r')
    params = load(yaml_file)
    csv_topics = params['topics']
    use_relative_time = params['relative_time']
    
    for fname in bags:
        csv_filename = fname.split('.')[0] + '.csv'    
        csvfile = open(csv_filename, 'w')
        csv_writer = csv.writer(csvfile, delimiter=',')
        
        # Load bag file    
        print 'Loading bag file: [' + fname + ']'
        bag = rosbag.Bag(fname)
        bag_types = bag.get_type_and_topic_info(csv_topics)[1].values()
        bag_topics = bag.get_type_and_topic_info(csv_topics)[1].keys()
        start_time = bag.get_start_time()
        
        if not use_relative_time:
            start_time = 0
        
        for csv_topic in csv_topics:
            for j in xrange(0, len(bag_topics)):
                if csv_topic == bag_topics[j]:
                    csv_types.append(bag_types[j].msg_type)
        
        print 'Topics being converted:'
        for i in xrange(0, len(csv_topics)):
            print '    ' + csv_topics[i] + ' (' + csv_types[i] + ')'
        
        
        # Initialize
        unsupported_topics = []
        num_messages = bag.get_message_count(csv_topics)
        message_count = 0.0
        percent_thres = 10.0
        
        # Convert
        print 'Converting...'
        for topic, msg, t in bag.read_messages(csv_topics):
            type_idx = get_csv_type(topic)
            if type_idx >= 0:
                # Found desired topic, determine type and write it to CSV
                if csv_types[type_idx] == BOOL_TYPE:
                    csv_fcns.write_bool_msg(t.to_sec()-start_time, topic, msg, csv_writer)                    
                elif csv_types[type_idx] == TWIST_TYPE:
                    csv_fcns.write_twist_msg(t.to_sec()-start_time, topic, msg, csv_writer)
                elif csv_types[type_idx] == TWISTSTAMPED_TYPE:
                    csv_fcns.write_twist_msg(t.to_sec()-start_time, topic, msg.twist, csv_writer)
                elif csv_types[type_idx] == FIX_TYPE:
                    csv_fcns.write_fix_msg(t.to_sec()-start_time, topic, msg, csv_writer)
                elif csv_types[type_idx] == IMU_TYPE:                
                    csv_fcns.write_imu_msg(t.to_sec()-start_time, topic, msg, csv_writer)
                elif csv_types[type_idx] == THROTTLE_REPORT_TYPE:
                    csv_fcns.write_throttle_report_msg(t.to_sec()-start_time, topic, msg, csv_writer)                
                elif csv_types[type_idx] == BRAKE_REPORT_TYPE:
                    csv_fcns.write_brake_report_msg(t.to_sec()-start_time, topic, msg, csv_writer)
                elif csv_types[type_idx] == STEERING_REPORT_TYPE:
                    csv_fcns.write_steering_report_msg(t.to_sec()-start_time, topic, msg, csv_writer)
                elif csv_types[type_idx] == GEAR_REPORT_TYPE:
                    csv_fcns.write_gear_report_msg(t.to_sec()-start_time, topic, msg, csv_writer)
                elif csv_types[type_idx] == FUEL_LEVEL_REPORT_TYPE:
                    csv_fcns.write_fuel_msg(t.to_sec()-start_time, topic, msg, csv_writer)
                elif csv_types[type_idx] == TIRE_PRESSURE_REPORT_TYPE:
                    csv_fcns.write_tire_pressure_msg(t.to_sec()-start_time, topic, msg, csv_writer)
                elif csv_types[type_idx] == WHEEL_SPEED_REPORT_TYPE:
                    csv_fcns.write_wheel_speed_msg(t.to_sec()-start_time, topic, msg, csv_writer)         
                elif csv_types[type_idx] == MISC1_REPORT_TYPE:
                    csv_fcns.write_misc1_msg(t.to_sec()-start_time, topic, msg, csv_writer)
                elif type_idx not in unsupported_topics:
                    print bcolors.WARNING + topic + ': Message type not supported by CSV converter (' + csv_types[type_idx] + ')' + bcolors.ENDC
                    unsupported_topics.append(type_idx)      
            message_count += 1
            if (100.0 * message_count / num_messages) >= percent_thres:
                percent_thres += 10
                print str(int(100 * message_count / num_messages)) + '% Complete'       
              
        print bcolors.OKGREEN + 'Finished conversion! Saved CSV data to [' + csv_filename + ']' + bcolors.ENDC
        csvfile.close()
    
if __name__ == '__main__':
    try:
        bag_to_csv()
    except rospy.ROSInterruptException:
        pass