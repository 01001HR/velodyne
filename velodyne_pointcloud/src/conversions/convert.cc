/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#include "convert.h"

#include <pcl_conversions/pcl_conversions.h>

namespace velodyne_pointcloud
{
    /** @brief Constructor. */
    Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
            data_(new velodyne_rawdata::RawData())
    {
      data_->setup(private_nh);

      // get model name, validate string, determine unpack
      std::string model;
      private_nh.param("model", model, std::string("VLP16"));

      // advertise output point cloud (before subscribing to input data)
      output_ =
              node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);

      srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_pointcloud::
      CloudNodeConfig> > (private_nh);
      dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig>::
      CallbackType f;
      f = boost::bind (&Convert::callback, this, _1, _2);
      srv_->setCallback (f);

      // subscribe to VelodyneScan packets
      if(model == "VLP16"){
        velodyne_scan_ =
                node.subscribe("velodyne_packets", 10,
                               &Convert::processScanVLP16, (Convert *) this,
                               ros::TransportHints().tcpNoDelay(true));
      }
      else{
        velodyne_scan_ =
                node.subscribe("velodyne_packets", 10,
                               &Convert::processScan, (Convert *) this,
                               ros::TransportHints().tcpNoDelay(true));
      }
    }

    void Convert::callback(velodyne_pointcloud::CloudNodeConfig &config,
                           uint32_t level)
    {
      ROS_INFO("Reconfigure Request");
      data_->setParameters(config.min_range, config.max_range, config.view_direction,
                           config.view_width);
    }

    /** @brief Callback for raw scan messages. */
    void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
    {
      if (output_.getNumSubscribers() == 0)         // no one listening?
        return;                                     // avoid much work

      // allocate a point cloud with same time and frame ID as raw data
      velodyne_rawdata::VPointCloud::Ptr
              outMsg(new velodyne_rawdata::VPointCloud());
      // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
      outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
      outMsg->header.frame_id = scanMsg->header.frame_id;
      outMsg->height = 1;

      // process each packet provided by the driver
      for (size_t i = 0; i < scanMsg->packets.size(); ++i)
      {
        data_->unpack(scanMsg->packets[i], *outMsg);
      }

      // publish the accumulated cloud message
      ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
                                     << " Velodyne points, time: " << outMsg->header.stamp);
      output_.publish(outMsg);
    }

    /** @brief Callback for VLP16 raw scan messages. */
    void Convert::processScanVLP16(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
    {
      if (output_.getNumSubscribers() == 0)         // no one listening?
        return;                                     // avoid much work

      // allocate a point cloud with same time and frame ID as raw data
      velodyne_rawdata::VTPointCloud::Ptr
              outMsg(new velodyne_rawdata::VTPointCloud());
      std::vector<velodyne_rawdata::VTPointCloud> laserCloudScans(16);

      // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
      outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
      outMsg->header.frame_id = scanMsg->header.frame_id;
      outMsg->height = 16;
      outMsg->width = scanMsg->packets.size() * 12 * 2;

      // process each packet provided by the driver
      for (size_t i = 0; i < scanMsg->packets.size(); ++i)
      {
        //data_->unpack(scanMsg->packets[i], *outMsg);
        data_->unpack_vlp16(scanMsg->packets[i], laserCloudScans);
      }

      for (int i = 0; i < 16; i++) {
        *outMsg += laserCloudScans[i];
      }

      // publish the accumulated cloud message
      ROS_DEBUG_STREAM("Publishing " << outMsg->height * outMsg->width
                                     << " Velodyne points, time: " << outMsg->header.stamp);
      output_.publish(outMsg);
    }

} // namespace velodyne_pointcloud
