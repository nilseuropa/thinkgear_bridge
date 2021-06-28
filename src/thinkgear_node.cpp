#include <ros/ros.h>
#include <ros/console.h>
#include <serial/serial.h>
#include <std_msgs/Float32.h>

ros::Publisher raw_signal_pub;
ros::Publisher signal_qual_pub;
ros::Publisher attention_pub;
ros::Publisher meditation_pub;

/* Lead in and out */
#define SYNC                 0xAA
#define EXCODE               0x55

/* Parser types */
#define TYPE_NULLTYPE_NULL   0x00
#define TYPE_PACKETS         0x01 /* Stream bytes as ThinkGear Packets */
#define TYPE_2BYTERAW        0x02 /* Stream bytes as 2-byte raw data */

/* Level-0 data CODE definitions */
#define BATTERY              0x01
#define POOR_SIGNAL          0x02 /* POOR_SIGNAL Quality (0-255) */
#define HEART_RATE           0x03 /* HEART_RATE (0-255) Once/s on EGO. */
#define ATTENTION            0x04 /* ATTENTION eSense (0 to 100) */
#define MEDITATION           0x05 /* MEDITATION eSense (0 to 100 */
#define RAW_EIGHTBIT         0x06 /* Raw wave value (0-255) */
#define RAW_MARKER           0x07 /* Section start (0) */

/* EEG band order: delta, theta, low-alpha, high-alpha, low-beta, high-beta, low-gamma, and mid-gamma */
#define RAW_EEG_SIGNAL       0x80 /* RAW Wave Value: a single big-endian 16-bit two's-compliment signed value (-32768 to 32767) */
#define EEG_POWER            0x81 /* eight big-endian 4-byte IEEE 754 floating point values representing EEG band power values */
#define ASIC_EEG_POWER       0x83 /* eight big-endian 3-byte unsigned integer values representing EEG band power values*/
#define RR_INTERVAL          0x86 /* two byte big-endian unsigned integer representing the milliseconds between two R-peaks */

int parsePayload( uint8_t *payload, uint8_t pLength ) {


    uint8_t bytesParsed = 0;
    uint8_t code;
    uint8_t length;
    uint8_t extendedCodeLevel;
    int i;

    /* Loop until all bytes are parsed from the payload[] array... */
    while( bytesParsed < pLength ) {

        extendedCodeLevel = 0;
        while( payload[bytesParsed] == EXCODE ) {
            extendedCodeLevel++;
            bytesParsed++;
        }
        code = payload[bytesParsed++];
        if( code & 0x80 ) length = payload[bytesParsed++];
        else              length = 1;

        // ROS_INFO( "EXCODE level: %d --- CODE: 0x%02X / length: %d", extendedCodeLevel, code, length );
        /*
        [ INFO] [1624895311.898170736]: EXCODE level: 0 --- CODE: 0x80 / length: 2
        [ INFO] [1624895311.904161209]: EXCODE level: 0 --- CODE: 0x02 / length: 1
        [ INFO] [1624895311.904168650]: EXCODE level: 0 --- CODE: 0x83 / length: 24
        [ INFO] [1624895311.904173860]: EXCODE level: 0 --- CODE: 0x04 / length: 1
        [ INFO] [1624895311.904178659]: EXCODE level: 0 --- CODE: 0x05 / length: 1
        */

        if ( code == POOR_SIGNAL){
          std_msgs::Float32 qual_msg;
          uint8_t signal_quality = payload[bytesParsed];
          qual_msg.data = signal_quality / 255.0f;
          signal_qual_pub.publish(qual_msg);
        }
        else if ( code == ATTENTION){
          std_msgs::Float32 attention_msg;
          uint8_t attention_level = payload[bytesParsed];
          attention_msg.data = attention_level / 100.0f;
          attention_pub.publish(attention_msg);
        }
        else if ( code == MEDITATION){
          std_msgs::Float32 meditation_msg;
          uint8_t meditation_level = payload[bytesParsed];
          meditation_msg.data = meditation_level / 100.0f;
          meditation_pub.publish(meditation_msg);
        }
        else if ( code == RAW_EEG_SIGNAL){
          std_msgs::Float32 raw_msg;
          int16_t raw_eeg = payload[bytesParsed] << 8 | payload[bytesParsed+1];
          raw_msg.data = raw_eeg/32768.0f;
          raw_signal_pub.publish(raw_msg);
        }

        /* Increment the bytesParsed by the length of the Data Value */
        bytesParsed += length;
    }

    return( 0 );
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "thinkgear_node");
    ros::NodeHandle n;
    ros::NodeHandle nhLocal("~");
    serial::Serial    thinkgear;
    std::string       port;
    int               baud;

    int checksum;
    uint8_t payload[256];
    uint8_t pLength;
    uint8_t c;

    nhLocal.param<std::string>("port", port, "/dev/ttyUSB0");
    nhLocal.param("baud", baud, 57600);

    serial::Timeout timeout = serial::Timeout::simpleTimeout(100);

    thinkgear.setPort(port);
    thinkgear.setBaudrate(baud);
    thinkgear.setTimeout(timeout);

    while ( ros::ok() ) {
        ROS_INFO_STREAM("Opening serial port on " << port << " at " << baud << "..." );
        try {
            thinkgear.open();
            if ( thinkgear.isOpen() )
            {
                ROS_INFO("Successfully opened serial port.");
                break;
            }
        }
        catch (serial::IOException e) {
            ROS_ERROR_STREAM("serial::IOException: " << e.what());
        }
        ROS_ERROR("Failed to open serial port");
        ROS_WARN("Retry in 5 seconds.");
        sleep( 5 );
    }

    ROS_INFO("Advertising EEG publishers.");
    raw_signal_pub  = n.advertise<std_msgs::Float32>("/eeg/raw", 100);
    signal_qual_pub = n.advertise<std_msgs::Float32>("/eeg/quality", 1);
    attention_pub   = n.advertise<std_msgs::Float32>("/eeg/attention", 1);
    meditation_pub  = n.advertise<std_msgs::Float32>("/eeg/meditation", 1);

    while (ros::ok()){
      ros::spinOnce();
      if (thinkgear.available()>0) {
        thinkgear.read((uint8_t*)&c, 1);
        if (c==SYNC&&thinkgear.available()>0){
          thinkgear.read((uint8_t*)&c, 1);
          if (c==SYNC&thinkgear.available()>0) {
            thinkgear.read((uint8_t*)&c, 1);
            if (!(c>=SYNC)) {
              pLength = c;
              while (thinkgear.available()<pLength);
              thinkgear.read((uint8_t*)&payload, pLength);
              parsePayload(payload, pLength);
            }
          }
        }
      }
    }

    if (thinkgear.isOpen()) thinkgear.close();
    ROS_INFO("ThinkGear node exiting...");
    return 0;
}
