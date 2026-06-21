#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <string>

#include <livox_ros_driver/CustomMsg.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

namespace
{

struct TimedImage
{
  sensor_msgs::ImagePtr msg;
  ros::Time arrival_time;
};

struct TimedLidar
{
  livox_ros_driver::CustomMsgPtr msg;
  ros::Time arrival_time;
};

class SoftTimeSync
{
public:
  SoftTimeSync()
    : nh_(),
      pnh_("~")
  {
    pnh_.param<std::string>("raw_lidar_topic", raw_lidar_topic_, "/livox/lidar");
    pnh_.param<std::string>("raw_imu_topic", raw_imu_topic_, "/livox/imu");
    pnh_.param<std::string>("raw_image_topic", raw_image_topic_, "/hikrobot_camera/rgb");
    pnh_.param<std::string>("synced_lidar_topic", synced_lidar_topic_, "/soft_sync/livox/lidar");
    pnh_.param<std::string>("synced_imu_topic", synced_imu_topic_, "/soft_sync/livox/imu");
    pnh_.param<std::string>("synced_image_topic", synced_image_topic_, "/soft_sync/camera/image");
    pnh_.param("match_wait_sec", match_wait_sec_, 0.10);
    pnh_.param("max_time_diff_sec", max_time_diff_sec_, 0.05);
    pnh_.param("buffer_duration_sec", buffer_duration_sec_, 1.0);

    match_wait_sec_ = std::max(0.0, match_wait_sec_);
    max_time_diff_sec_ = std::max(0.0, max_time_diff_sec_);
    buffer_duration_sec_ = std::max(buffer_duration_sec_, match_wait_sec_ + max_time_diff_sec_);

    lidar_pub_ = nh_.advertise<livox_ros_driver::CustomMsg>(synced_lidar_topic_, 100);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>(synced_imu_topic_, 1000);
    image_pub_ = nh_.advertise<sensor_msgs::Image>(synced_image_topic_, 100);

    lidar_sub_ = nh_.subscribe(raw_lidar_topic_, 100, &SoftTimeSync::lidarCallback, this);
    imu_sub_ = nh_.subscribe(raw_imu_topic_, 1000, &SoftTimeSync::imuCallback, this);
    image_sub_ = nh_.subscribe(raw_image_topic_, 100, &SoftTimeSync::imageCallback, this);

    const double timer_period = std::max(0.001, std::min(0.01, match_wait_sec_ * 0.25));
    match_timer_ = nh_.createWallTimer(ros::WallDuration(timer_period), &SoftTimeSync::matchTimerCallback, this);

    ROS_INFO_STREAM("FAST-LIVO2 soft time sync started."
                    << "\n  raw lidar:   " << raw_lidar_topic_
                    << "\n  raw imu:     " << raw_imu_topic_
                    << "\n  raw image:   " << raw_image_topic_
                    << "\n  synced lidar:" << synced_lidar_topic_
                    << "\n  synced imu:  " << synced_imu_topic_
                    << "\n  synced image:" << synced_image_topic_
                    << "\n  match wait: " << match_wait_sec_ << " s"
                    << ", max difference: " << max_time_diff_sec_ << " s");
  }

private:
  static ros::Time systemNow()
  {
    const ros::WallTime wall_time = ros::WallTime::now();
    return ros::Time(wall_time.sec, wall_time.nsec);
  }

  void lidarCallback(const livox_ros_driver::CustomMsg::ConstPtr& input)
  {
    const ros::Time arrival_time = systemNow();
    livox_ros_driver::CustomMsgPtr output(new livox_ros_driver::CustomMsg(*input));

    // Livox offset_time is measured from the frame header in nanoseconds.
    // Treat the callback arrival time as the frame-end system time and recover
    // a frame-start stamp so FAST-LIVO2 can continue using per-point timing.
    std::uint32_t max_offset_ns = 0;
    for (const auto& point : output->points)
    {
      max_offset_ns = std::max(max_offset_ns, point.offset_time);
    }
    output->header.stamp = arrival_time - ros::Duration(static_cast<double>(max_offset_ns) * 1e-9);

    std::lock_guard<std::mutex> lock(buffer_mutex_);
    lidar_buffer_.push_back({output, arrival_time});
    trimBuffers(arrival_time);
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& input)
  {
    sensor_msgs::Imu output(*input);
    output.header.stamp = systemNow();
    imu_pub_.publish(output);
  }

  void imageCallback(const sensor_msgs::Image::ConstPtr& input)
  {
    const ros::Time arrival_time = systemNow();
    sensor_msgs::ImagePtr output(new sensor_msgs::Image(*input));
    output->header.stamp = arrival_time;

    std::lock_guard<std::mutex> lock(buffer_mutex_);
    image_buffer_.push_back({output, arrival_time});
    trimBuffers(arrival_time);
  }

  void matchTimerCallback(const ros::WallTimerEvent&)
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    const ros::Time now = systemNow();

    while (!lidar_buffer_.empty())
    {
      const TimedLidar& lidar = lidar_buffer_.front();
      if ((now - lidar.arrival_time).toSec() < match_wait_sec_) break;

      if (image_buffer_.empty())
      {
        dropLidar("no image available");
        continue;
      }

      auto best_image = image_buffer_.end();
      double best_difference = std::numeric_limits<double>::infinity();
      for (auto it = image_buffer_.begin(); it != image_buffer_.end(); ++it)
      {
        const double difference = std::abs((it->arrival_time - lidar.arrival_time).toSec());
        if (difference < best_difference)
        {
          best_difference = difference;
          best_image = it;
        }
      }

      if (best_image == image_buffer_.end() || best_difference > max_time_diff_sec_)
      {
        dropLidar("nearest image exceeds max_time_diff_sec");
        continue;
      }

      // One LiDAR frame consumes exactly one image. Both are aligned to the
      // LiDAR frame-end system time; the LiDAR header itself remains the
      // recovered frame-start time for point-wise undistortion.
      sensor_msgs::Image matched_image(*best_image->msg);
      matched_image.header.stamp = lidar.arrival_time;

      lidar_pub_.publish(*lidar.msg);
      image_pub_.publish(matched_image);

      ++matched_pairs_;
      ROS_INFO_STREAM_THROTTLE(2.0, "Soft sync matched pairs: " << matched_pairs_
                                      << ", last arrival-time difference: "
                                      << best_difference * 1000.0 << " ms"
                                      << ", dropped LiDAR frames: " << dropped_lidar_frames_);

      image_buffer_.erase(best_image);
      lidar_buffer_.pop_front();
      trimBuffers(now);
    }
  }

  void dropLidar(const char* reason)
  {
    ++dropped_lidar_frames_;
    lidar_buffer_.pop_front();
    ROS_WARN_STREAM_THROTTLE(2.0, "Soft sync dropped LiDAR frame (" << reason
                                      << "). Total dropped: " << dropped_lidar_frames_);
  }

  void trimBuffers(const ros::Time& now)
  {
    while (!image_buffer_.empty() &&
           (now - image_buffer_.front().arrival_time).toSec() > buffer_duration_sec_)
    {
      image_buffer_.pop_front();
      ++dropped_image_frames_;
    }

    while (!lidar_buffer_.empty() &&
           (now - lidar_buffer_.front().arrival_time).toSec() > buffer_duration_sec_)
    {
      lidar_buffer_.pop_front();
      ++dropped_lidar_frames_;
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber lidar_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber image_sub_;
  ros::Publisher lidar_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher image_pub_;
  ros::WallTimer match_timer_;

  std::string raw_lidar_topic_;
  std::string raw_imu_topic_;
  std::string raw_image_topic_;
  std::string synced_lidar_topic_;
  std::string synced_imu_topic_;
  std::string synced_image_topic_;

  double match_wait_sec_;
  double max_time_diff_sec_;
  double buffer_duration_sec_;

  std::mutex buffer_mutex_;
  std::deque<TimedImage> image_buffer_;
  std::deque<TimedLidar> lidar_buffer_;

  std::uint64_t matched_pairs_ = 0;
  std::uint64_t dropped_lidar_frames_ = 0;
  std::uint64_t dropped_image_frames_ = 0;
};

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fast_livo_soft_time_sync");
  SoftTimeSync sync;
  ros::spin();
  return 0;
}
