/*
 * This file is part of lslidar_c32 driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LSLIDAR_C32_DECODER_H
#define LSLIDAR_C32_DECODER_H

#define DEG_TO_RAD 0.017453292 
#define RAD_TO_DEG 57.29577951

#include <cmath>
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <lslidar_c32_msgs/LslidarC32Packet.h>
#include <lslidar_c32_msgs/LslidarC32Point.h>
#include <lslidar_c32_msgs/LslidarC32Scan.h>
#include <lslidar_c32_msgs/LslidarC32Sweep.h>
#include <lslidar_c32_msgs/LslidarC32Layer.h>


namespace lslidar_c32_decoder {

// Raw lslidar packet constants and structures.
static const int SIZE_BLOCK      = 100;
static const int RAW_SCAN_SIZE   = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE =
        (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

// According to Bruce Hall DISTANCE_MAX is 65.0, but we noticed
// valid packets with readings up to 130.0.
static const double DISTANCE_MAX        = 130.0;        /**< meters */
static const double DISTANCE_RESOLUTION = 0.01; /**< meters */
static const double DISTANCE_MAX_UNITS  =
        (DISTANCE_MAX / DISTANCE_RESOLUTION + 1.0);

/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines for VLP16 support **/
static const int     FIRINGS_PER_BLOCK = 1;
static const int     SCANS_PER_FIRING  = 32;
static const double  BLOCK_TDURATION   = 110.592; // [µs]
static const double  DSR_TOFFSET       = 2.304;   // [µs]
static const double  FIRING_TOFFSET    = 55.296;  // [µs]

static const int PACKET_SIZE        = 1206;
static const int BLOCKS_PER_PACKET  = 12;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET =
        (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);
static const int FIRINGS_PER_PACKET =
        FIRINGS_PER_BLOCK * BLOCKS_PER_PACKET;

// Pre-compute the sine and cosine for the altitude angles.
/*
static const double scan_altitude[16] = {
    -0.2617993877991494,   0.017453292519943295,
    -0.22689280275926285,  0.05235987755982989,
    -0.19198621771937624,  0.08726646259971647,
    -0.15707963267948966,  0.12217304763960307,
    -0.12217304763960307,  0.15707963267948966,
    -0.08726646259971647,  0.19198621771937624,
    -0.05235987755982989,  0.22689280275926285,
    -0.017453292519943295, 0.2617993877991494
};

static const double scan_altitude[32] = {
    -0.28797932657906438019240897680062,-0.04188790204786390984616857844373,
    -0.27925268031909273230779052295818,-0.0314159265358979323846264338328,
    -0.25307274153917778865393516143085,-0.02199114857512855266923850368296,
    -0.24434609527920614076931670758841,-0.01221730476396030703846583537942,
    -0.21816615649929119711546134606108,-0.00244346095279206140769316707588,
    -0.20943951023931954923084289221863,0.0075049157835756171807718703045,
    -0.1832595714594046055769875306913,0.01727875959474386281154453860804,
    -0.17453292519943295769236907684886,0.02705260340591210844231720691157,
    -0.14660765716752368446159002455304,0.04712388980384689857693965074919,
    -0.1378810109075520365769715707106,0.05759586531581287603848179536012,
    -0.11170107212763709292311620918327,0.08377580409572781969233715688745,
    -0.10122909661567111546157406457234,0.09424777960769379715387930149839,
    -0.08028514559173916053848977535048,0.11868238913561441123081097225723,
    -0.06981317007977318307694763073954,0.12915436464758038869235311686816,
    -0.0610865238198015351923291768971,0.15533430342749533234620847839549,
    -0.05061454830783555773078703228617,0.16406094968746698023082693223793
};*/
/*static const double layer_altitude[32] = {
    -0.28797932657906438019240897680062,
    -0.25307274153917778865393516143085,
    -0.21816615649929119711546134606108,
    -0.1832595714594046055769875306913,
    -0.14660765716752368446159002455304,
    -0.11170107212763709292311620918327,
    -0.08028514559173916053848977535048,
    -0.0610865238198015351923291768971,
    -0.04188790204786390984616857844373,
    -0.02199114857512855266923850368296,
    -0.00244346095279206140769316707588,
    0.01727875959474386281154453860804,
     0.04712388980384689857693965074919,
     0.08377580409572781969233715688745,
     0.11868238913561441123081097225723,
    0.15533430342749533234620847839549,
    -0.27925268031909273230779052295818,
    -0.24434609527920614076931670758841,
    -0.20943951023931954923084289221863,
    -0.17453292519943295769236907684886,
    -0.1378810109075520365769715707106,
    -0.10122909661567111546157406457234,
    -0.06981317007977318307694763073954,
    -0.05061454830783555773078703228617,
    -0.0314159265358979323846264338328,
    -0.01221730476396030703846583537942,
    0.0075049157835756171807718703045,
    0.02705260340591210844231720691157,
   0.05759586531581287603848179536012,
    0.09424777960769379715387930149839,
    0.12915436464758038869235311686816,
    0.16406094968746698023082693223793
};

static const double scan_altitude[32] = {
        -0.28797932657906438019240897680062,-0.27925268031909273230779052295818,
        -0.25307274153917778865393516143085,-0.24434609527920614076931670758841,
        -0.21816615649929119711546134606108,-0.20943951023931954923084289221863,
        -0.1832595714594046055769875306913,-0.17453292519943295769236907684886,
        -0.14660765716752368446159002455304,-0.1378810109075520365769715707106,
        -0.11170107212763709292311620918327,-0.10122909661567111546157406457234,
        -0.08028514559173916053848977535048,-0.06981317007977318307694763073954,
        -0.0610865238198015351923291768971,-0.05061454830783555773078703228617,
        -0.04188790204786390984616857844373,-0.0314159265358979323846264338328,
        -0.02199114857512855266923850368296,-0.01221730476396030703846583537942,
        -0.00244346095279206140769316707588,0.0075049157835756171807718703045,
        0.01727875959474386281154453860804,0.02705260340591210844231720691157,
        0.04712388980384689857693965074919,0.05759586531581287603848179536012,
        0.08377580409572781969233715688745,0.09424777960769379715387930149839,
        0.11868238913561441123081097225723,0.12915436464758038869235311686816,
        0.15533430342749533234620847839549,0.16406094968746698023082693223793
};*/


static const double scan_altitude[32] = {
        -0.2792526803190927,-0.2617993877991494,
        -0.2443460952792061,-0.2268928027592628,
        -0.2094395102393195,-0.1919862177193762,
        -0.1745329251994329,-0.1570796326794897,
        -0.1396263401595464,-0.1221730476396031,
        -0.1047197551196598,-0.0872664625997165,
        -0.0698131700797732,-0.0523598775598299,
        -0.0349065850398866,-0.0174532925199433,
        0.0, 0.0174532925199433,
        0.0349065850398866, 0.0523598775598299,
        0.0698131700797732, 0.0872664625997165,
        0.1047197551196598, 0.1221730476396031,
        0.1396263401595464, 0.1570796326794897,
        0.1745329251994329, 0.1919862177193762,
        0.2094395102393195, 0.2268928027592628,
        0.2443460952792061, 0.2617993877991494
};

static const double layer_altitude[32] = {
            scan_altitude[ 0],
            scan_altitude[ 2],
            scan_altitude[ 4],
            scan_altitude[ 6],
            scan_altitude[ 8],
            scan_altitude[10],
            scan_altitude[12],
            scan_altitude[14],
            scan_altitude[16],
            scan_altitude[18],
            scan_altitude[20],
            scan_altitude[22],
            scan_altitude[24],
            scan_altitude[26],
            scan_altitude[28],
            scan_altitude[30],
            scan_altitude[ 1],
            scan_altitude[ 3],
            scan_altitude[ 5],
            scan_altitude[ 7],
            scan_altitude[ 9],
            scan_altitude[11],
            scan_altitude[13],
            scan_altitude[15],
            scan_altitude[17],
            scan_altitude[19],
            scan_altitude[21],
            scan_altitude[23],
            scan_altitude[25],
            scan_altitude[27],
            scan_altitude[29],
            scan_altitude[31]
    };

//static const int layer_id[32] = {1,17,2,18,3,19,4,20,5,21,6,22,7,23,8,24,9,25,10,26,11,27,12,28,13,29,14,30,15,31,16,32};
static const int layer_id[32] = {1,3,5,7,9,11,13,15,17,19,21,23,25,27,29,31,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32};



static const double cos_scan_altitude[32] = {
    std::cos(scan_altitude[ 0]), std::cos(scan_altitude[ 1]),
    std::cos(scan_altitude[ 2]), std::cos(scan_altitude[ 3]),
    std::cos(scan_altitude[ 4]), std::cos(scan_altitude[ 5]),
    std::cos(scan_altitude[ 6]), std::cos(scan_altitude[ 7]),
    std::cos(scan_altitude[ 8]), std::cos(scan_altitude[ 9]),
    std::cos(scan_altitude[10]), std::cos(scan_altitude[11]),
    std::cos(scan_altitude[12]), std::cos(scan_altitude[13]),
    std::cos(scan_altitude[14]), std::cos(scan_altitude[15]),
    std::cos(scan_altitude[16]), std::cos(scan_altitude[17]),
    std::cos(scan_altitude[18]), std::cos(scan_altitude[19]),
    std::cos(scan_altitude[20]), std::cos(scan_altitude[21]),
    std::cos(scan_altitude[22]), std::cos(scan_altitude[23]),
    std::cos(scan_altitude[24]), std::cos(scan_altitude[25]),
    std::cos(scan_altitude[26]), std::cos(scan_altitude[27]),
    std::cos(scan_altitude[28]), std::cos(scan_altitude[29]),
    std::cos(scan_altitude[30]), std::cos(scan_altitude[31]),
};

static const double sin_scan_altitude[32] = {
    std::sin(scan_altitude[ 0]), std::sin(scan_altitude[ 1]),
    std::sin(scan_altitude[ 2]), std::sin(scan_altitude[ 3]),
    std::sin(scan_altitude[ 4]), std::sin(scan_altitude[ 5]),
    std::sin(scan_altitude[ 6]), std::sin(scan_altitude[ 7]),
    std::sin(scan_altitude[ 8]), std::sin(scan_altitude[ 9]),
    std::sin(scan_altitude[10]), std::sin(scan_altitude[11]),
    std::sin(scan_altitude[12]), std::sin(scan_altitude[13]),
    std::sin(scan_altitude[14]), std::sin(scan_altitude[15]),
    std::sin(scan_altitude[16]), std::sin(scan_altitude[17]),
    std::sin(scan_altitude[18]), std::sin(scan_altitude[19]),
    std::sin(scan_altitude[20]), std::sin(scan_altitude[21]),
    std::sin(scan_altitude[22]), std::sin(scan_altitude[23]),
    std::sin(scan_altitude[24]), std::sin(scan_altitude[25]),
    std::sin(scan_altitude[26]), std::sin(scan_altitude[27]),
    std::sin(scan_altitude[28]), std::sin(scan_altitude[29]),
    std::sin(scan_altitude[30]), std::sin(scan_altitude[31]),
};

typedef struct{
    double distance;
    double intensity;
}point_struct;

struct PointXYZIT {
    PCL_ADD_POINT4D
    float intensity;
    float v_angle;
    float h_angle;
    float range;
    int laserid;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;

class LslidarC32Decoder {
public:

    LslidarC32Decoder(ros::NodeHandle& n, ros::NodeHandle& pn);
    LslidarC32Decoder(const LslidarC32Decoder&) = delete;
    LslidarC32Decoder operator=(const LslidarC32Decoder&) = delete;
    ~LslidarC32Decoder() {return;}

    bool initialize();

    typedef boost::shared_ptr<LslidarC32Decoder> LslidarC32DecoderPtr;
    typedef boost::shared_ptr<const LslidarC32Decoder> LslidarC32DecoderConstPtr;

private:

    union TwoBytes {
        uint16_t distance;
        uint8_t  bytes[2];
    };

    struct RawBlock {
        uint16_t header;        ///< UPPER_BANK or LOWER_BANK
        uint16_t rotation;      ///< 0-35999, divide by 100 to get degrees
        uint8_t  data[BLOCK_DATA_SIZE];
    };

    struct RawPacket {
        RawBlock blocks[BLOCKS_PER_PACKET];
        uint32_t time_stamp;
        uint8_t factory[2];
        //uint16_t revolution;
        //uint8_t status[PACKET_STATUS_SIZE];
    };

    struct Firing {
        // Azimuth associated with the first shot within this firing.
        double firing_azimuth;
        double azimuth[SCANS_PER_FIRING];
        double distance[SCANS_PER_FIRING];
        double intensity[SCANS_PER_FIRING];
    };

    // Intialization sequence
    bool loadParameters();
    bool createRosIO();


    // Callback function for a single lslidar packet.
    bool checkPacketValidity(const RawPacket* packet);
    void decodePacket(const RawPacket* packet);
    void layerCallback(const std_msgs::Int8Ptr& msg);
    void packetCallback(const lslidar_c32_msgs::LslidarC32PacketConstPtr& msg);
    // Publish data
    void publishPointCloud();
    void publishChannelScan();
    // Publish scan Data
    void publishScan();

    // Check if a point is in the required range.
    bool isPointInRange(const double& distance) {
        return (distance >= min_range && distance <= max_range);
    }

    double rawAzimuthToDouble(const uint16_t& raw_azimuth) {
        // According to the user manual,
        // azimuth = raw_azimuth / 100.0;
        return static_cast<double>(raw_azimuth) / 100.0 * DEG_TO_RAD;
    }

    // calc the means_point
    point_struct getMeans(std::vector<point_struct> clusters);

    // configuration degree base
    int point_num;
    double angle_base;

    // Configuration parameters
    double min_range;
    double max_range;
    double angle_disable_min;
    double angle_disable_max;
    double frequency;
    bool publish_point_cloud;
    bool publish_channels;
    
    double cos_azimuth_table[6300];
    double sin_azimuth_table[6300];

    bool is_first_sweep;
    double last_azimuth;
    double sweep_start_time;
    double packet_start_time;
    double point_time;
    int layer_num;
    Firing firings[FIRINGS_PER_PACKET];

    // ROS related parameters
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    std::string fixed_frame_id;
    std::string child_frame_id;

    lslidar_c32_msgs::LslidarC32SweepPtr sweep_data;
    lslidar_c32_msgs::LslidarC32LayerPtr multi_scan;
    sensor_msgs::PointCloud2 point_cloud_data;

    ros::Subscriber packet_sub;
    ros::Subscriber layer_sub;
    ros::Publisher sweep_pub;
    ros::Publisher point_cloud_pub;
    ros::Publisher scan_pub;
    ros::Publisher channel_scan_pub;

};

typedef LslidarC32Decoder::LslidarC32DecoderPtr LslidarC32DecoderPtr;
typedef LslidarC32Decoder::LslidarC32DecoderConstPtr LslidarC32DecoderConstPtr;
    typedef PointXYZIT VPoint;
    typedef pcl::PointCloud<VPoint> VPointCloud;
} // end namespace lslidar_c32_decoder
POINT_CLOUD_REGISTER_POINT_STRUCT(lslidar_c32_decoder::PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)
                                          (float, intensity, intensity) (float, v_angle, v_angle)
                                          (float, h_angle, h_angle)(float, range, range)(int, laserid, laserid))
#endif
