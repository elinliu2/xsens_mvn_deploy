#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include "header.h"

#define SERVER_PORT 9763
#define BUFF_LEN 1024

#define PI 3.141592


void parse_header(Header *header, char *buf, int *time_stamp_sec, int *time_stamp_nsec)
{
    int hour = 0;
    int min = 0;
    int sec = 0;
    int nanosec = 0;
    float a = 0.0;
    int to_stamp_sec = 0;
    float nano_1000;
    float temp;
    float float_time_code;
    double b = 0.0;
    memcpy(header->ID_String, buf, 6);
    memcpy(&(header->sample_counter), buf+6, 4);
    header->sample_counter = ntohl(header->sample_counter);
    header->datagram_counter = buf[11];
    header->number_of_items = buf[12];
    memcpy(&(header->time_code), buf+12, 4);
    header->time_code = ntohl(header->time_code);
    float_time_code = (float)header->time_code;
    //decoding time_code;
    if (header->time_code <= 99999) { //if time_code < 99.999;
	float_time_code /= 1000;
	//(debug) printf("float_time_code = header->time_code/1000 = %f\n", float_time_code);
	sec = (int)float_time_code;
	//(debug) printf("sec = (int)float_time_code = %d\n", sec);
	nanosec = (int)((float_time_code - sec)*1000);
	//(debug)printf("nanosec = %d\n", nanosec);
	to_stamp_sec = sec;
		
	//**convert to time stamp**
	*time_stamp_sec = to_stamp_sec;
	*time_stamp_nsec = nanosec;
    } else if (header->time_code > 99999 && header->time_code <= 9999999) { //if 99.999 < time_code <= 99:99.999;
	float_time_code = float_time_code/100000;
	min = (int)float_time_code;
	a = (int)((float_time_code - min)*100000);
	float_time_code = (float)a;
	float_time_code /= 1000;
	sec = (int)float_time_code;
	nanosec = (int)((float_time_code - sec)*1000);
	to_stamp_sec = sec + min*100;
	/* wrong logic!!
	   nanosec = (int)((float_time_code - a)*1000); 
	   float_time_code = (float)a;
	   float_time_code /= 100;
	   min = (int)float_time_code;
	   sec = (int)((float_time_code - min)*100);
	   to_stamp_sec = sec + min*60; */
	//**convert to time stamp**
	*time_stamp_sec = to_stamp_sec;
	*time_stamp_nsec = nanosec;
    } else if (header->time_code > 9999999 && header->time_code <= 999999999) { //if 99:99.999 < time_code < 99:99:99.999;
	float_time_code /= 10000000;
	hour = (int)float_time_code;
	a = (int)((float_time_code - hour)*10000000);
	float_time_code = (float)a;
	float_time_code = float_time_code/100000;
	min = (int)float_time_code;
	a = (int)((float_time_code - min)*100000);
	float_time_code = (float)a;
	float_time_code /= 1000;
	sec = (int)float_time_code;
	nanosec = (int)((float_time_code - sec)*1000);
	to_stamp_sec = sec + min*100 + hour*100*100;
	/*wrong logic!!
	  nanosec = (int)((float_time_code - a)*1000);
	  float_time_code = (float)a;
	  float_time_code /= 100;
	  min = (int)float_time_code;
	  sec = (int)((float_time_code - min)*100);
	  header->time_code = (int)header->time_code;
	  header->time_code = header->time_code/100;
	  hour = (int)header->time_code;
	  min = (header->time_code - hour)*100;
	  sec = sec + min*60 + hour*60*60; */
	//convert to time stamp
	*time_stamp_sec = to_stamp_sec;
	*time_stamp_nsec = nanosec;
    }
    header->character_ID = buf[17];
    memcpy(header->reserved_for_future_use, buf+18, 7);
    /* printf("ID_String = %s\n", header->ID_String);
    printf("sample_counter = %d\n", header->sample_counter);   //endian conversion
    printf("datagram_counter = %d\n", header->datagram_counter);  
    printf("number_of_items = %d\n", header->number_of_items);
    printf("time_code = %d\n", (int)header->time_code);
    printf("time = %d:%d:%d.%d\n", hour, min, sec, nanosec);
    printf("time stamp for rosmsg: %d.%d\n", *stamp_sec, *stamp_nsec);
    printf("character_ID = %d\n", header->character_ID); */
	
}



float parse_coordinates(float coordinate, int count, char *buf)
{
    // check Endianess
    //**** Intel processors use "Little Endian" byte order!
    int num = 1;
    int Endianess;
    if(*(char *)&num == 1) {
	Endianess = 0; //Little_Endian;
    } else {
	Endianess = 1; //Big_Endian;
    }
   
    unsigned char byte_array[4];
    memcpy(&byte_array, buf+count, sizeof(coordinate));
    
    if (Endianess == 0) {
	*((unsigned char*)(&coordinate) + 3) = byte_array[0];
	*((unsigned char*)(&coordinate) + 2) = byte_array[1];
	*((unsigned char*)(&coordinate) + 1) = byte_array[2];
	*((unsigned char*)(&coordinate) + 0) = byte_array[3];
    } else if (Endianess == 1) {
	*((unsigned char*)(&coordinate) + 3) = byte_array[3];
	*((unsigned char*)(&coordinate) + 2) = byte_array[2];
	*((unsigned char*)(&coordinate) + 1) = byte_array[1];
	*((unsigned char*)(&coordinate) + 0) = byte_array[0];
    }
    return coordinate;
}

void convertFromYupToZup(float *x, float *y, float *z)
{
    float x1 = *x;
    float y1 = *y;
    float z1 = *z;
	
    *x = z1;
    *y = x1;
    *z = y1;
}

float convertFromRadToDeg(float rad)
{
    float deg = rad*180/PI;
    return deg;
}

void parse_body(char *buf, int *segment_id, float *x, float *y, float *z, float *re, float *i, float *j, float *k)
{
    int seg_id = 0;
    float x_p, y_p, z_p = 0.0;
    float q1_r, q2_r, q3_r, q4_r = 0.0;
    memcpy(&seg_id, buf, 4);
    seg_id = ntohl(seg_id);
    *segment_id = seg_id;
    x_p = parse_coordinates(x_p, 4, buf);
    *x = x_p;
    //memcpy(&x_p, buf+4, 4);
    //x_p = ntohl(x_p);
    y_p = parse_coordinates(y_p, 8, buf);
    *y = y_p;
    //memcpy(&y_p, buf+8, 4); 	
    //y_p = ntohl(y_p);
    z_p = parse_coordinates(z_p, 12, buf);
    *z = z_p;
    //memcpy(&z_p, buf+12, 4); 	
    //z_p = ntohl(z_p);
    //convertFromYupToZup(&x_p, &y_p, &z_p);
    //need to convert from Y up tp Z up in type 01
	
    q1_r = parse_coordinates(q1_r, 16, buf);
    q1_r = convertFromRadToDeg(q1_r);
    *re = q1_r;
   
    q2_r = parse_coordinates(q2_r, 20, buf);
    q2_r = convertFromRadToDeg(q2_r);
    *i = q2_r;
   
    q3_r = parse_coordinates(q3_r, 24, buf);
    q3_r = convertFromRadToDeg(q3_r);
    *j = q3_r;
   
    q4_r = parse_coordinates(q4_r, 28, buf);
    q4_r = convertFromRadToDeg(q4_r);
    *k = q4_r;

    /********
    printf("Segment ID: %d\n", seg_id);
    printf("Segment Position: (%f, %f, %f)\n", x_p, y_p, z_p);
    printf("Segment Rotation Quaternion: (re:%f, i:%f, j:%f, k:%f)\n", q1_r, q2_r, q3_r, q4_r);
    ********/
}




void handle_udp_msg(int fd, int argc, char* argv[])
{
    char *buf = (char*)malloc(1024);
    int read_bytes = 0; 
    int cur_index = 0;
    int count;
    int time_stamp_sec;
    int time_stamp_nsec;
    int segment_id;
    float x, y, z;
    float re, i, j, k;
    struct sockaddr_in client_addr;
    socklen_t len = sizeof(client_addr);
    Header header;
    int round = 0;

    ros::init(argc, argv, "xsens_data_publisher_tf"); //name of the node
    ros::NodeHandle n;
    //ros::Publisher data_publisher = n.advertise<sensor_msgs::JointState>("position_data", 50);
    tf::TransformBroadcaster pelvis, l5, l3, t12, t8, neck, head, right_shoulder, right_upper_arm, right_forearm, right_hand, left_shoulder, left_upper_arm, left_forearm, left_hand, right_upper_leg, right_lower_leg, right_foot, right_toe, left_upper_leg, left_lower_leg, left_foot, left_toe;
   
    while(ros::ok) 
    {
	while(1) {
	    memset(buf, 0, BUFF_LEN);
	    count = recvfrom(fd, buf+read_bytes, BUFF_LEN, 0, (struct sockaddr*)&client_addr, &len);
	    if(count == -1)
		{
		    printf("receieve data fail!\n");
		    return;
		}
	    read_bytes += count;
	    if (read_bytes >= 24) {
		parse_header(&header, buf+cur_index, &time_stamp_sec, &time_stamp_nsec);
		cur_index += 24;
		if (strncmp(header.ID_String, "MXTP02", 6) == 0) {
		    //Message type 02 - Pose data (Quaternion)
		    if (read_bytes >= 24+32*header.datagram_counter) {
			int p = 0;
			//printf("Message type 02: Pose data (Quaternion)\n"); 
			for (p = 0; p < header.datagram_counter; p++) {
			    parse_body(buf+cur_index+p*32, &segment_id,  &x, &y, &z, &re, &i, &j, &k);
			    printf("re = %f, i = %f, j = %f, k = %f\n", re, i, j, k);
			    ros::Time temp;	    
			    switch (segment_id) {
			    case 1:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
				pelvis.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sonser", "pelvis"));
				break;
			    case 2:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
				l5.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sonser", "l5"));
				break;
			    case 3:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			        l3.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sonser", "l3"));
				break;
			    case 4:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			        t12.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sonser", "t12"));
				break;
			    case 5:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			        t8.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sonser", "t8"));
				break;
			    case 6:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			        neck.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sonser", "neck"));
				break;
			    case 7:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			        head.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sonser", "head"));
				break;
			    case 8:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			        right_shoulder.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sonser", "right_shoulder"));
				break;
			    case 9:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			        right_upper_arm.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sonser", "right_upper_arm"));
				break;
			    case 10:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			        right_forearm.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sonser", "right_forearm"));
				break;
			    case 11:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			        right_hand.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sonser", "right_hand"));
				break;
			    case 12:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			        left_shoulder.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sonser", "left_shoulder"));
				break;
			    case 13:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			        left_upper_arm.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sonser", "left_upper_arm"));
				break;
			    case 14:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			        left_forearm.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sonser", "left_forearm"));
				break;
			    case 15:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			        left_hand.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sonser", "left_hand"));
				break;
				break;
			    case 16:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			        right_upper_leg.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sonser", "right_upper_leg"));
				break;
			    case 17:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			        right_lower_leg.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sonser", "right_lower_leg"));
				break;
			    case 18:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			        right_foot.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sonser", "right_foot"));
				break;
			    case 19:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
			        right_toe.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sonser", "right_toe"));
				break;
			    case 20:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
				left_upper_leg.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sonser", "left_upper_leg"));
				break;
			    case 21:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
				left_lower_leg.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sonser", "left_lower_leg"));
				break;
			    case 22:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
				left_foot.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sonser", "left_foot"));
				break;
			    case 23:
				temp.sec = time_stamp_sec;
				temp.nsec = time_stamp_nsec;
				left_toe.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(i, j, k, re), tf::Vector3(x, y, z)), temp, "body_sonser", "left_toe"));
				break;
			    }
			}
			cur_index += 32*header.datagram_counter;
			int left = read_bytes - cur_index;
			//printf("bytes left to parse = %d\n", left);
			if (left > 0)
			    memcpy(buf, buf+cur_index, left);
			read_bytes = left;
			cur_index = 0;
		    } else {
			cur_index -= 24;
		    }
		} else {
		    //TODO
		    printf("other type\n");
		}
		//break;
	    }		
	    round++;
	    printf("round = %d\n", round);
	    //if (round == 2)
	    //break;
	    //parse_data(buf, &index, &parse_bytes);
	    //memset(buf, 0, BUFF_LEN);
	}
    }
    free(buf);
}


/*
  server:
  socket-->bind-->recvfrom-->sendto-->close
*/

int main(int argc, char* argv[])
{
    //ros::init(argc, argv, "xsens_data_publisher"); //name of the node
    //ros::NodeHandle n;
    //ros::Publisher data_publisher = n.advertise<sensor_msgs::JointState>("position_data", 50);
    
    int server_fd, ret;
    struct sockaddr_in ser_addr;
	
    server_fd = socket(AF_INET, SOCK_DGRAM, 0); //AF_INET: IPV4; SOCK_DGRAM: UDP
    if(server_fd < 0)
	{
	    printf("create socket fail!\n");
	    return -1;
	}
	
    memset(&ser_addr, 0, sizeof(ser_addr));
    ser_addr.sin_family = AF_INET;
    ser_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    ser_addr.sin_port = htons(SERVER_PORT);
	
    ret = bind(server_fd, (struct sockaddr*)&ser_addr, sizeof(ser_addr));
    if(ret < 0)
	{
	    printf("socket bind fail!\n");
	    return -1;
	}
		
    handle_udp_msg(server_fd, argc, argv);
	
    close(server_fd);




}
	
