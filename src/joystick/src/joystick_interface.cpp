// Copyright 2017 - 2018 slane@cs.umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
//========================================================================
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
//========================================================================

#include "joystick/joystick_interface.hpp"

#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/poll.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <iostream>

namespace joystick
{

JoystickInterface::JoystickInterface(const std::string & device_path)
: fd_(-1), device_path_(device_path)
{
}

JoystickInterface::~JoystickInterface()
{
  close();
}

bool JoystickInterface::open()
{
  close();  // Close any existing connection
  
  fd_ = ::open(device_path_.c_str(), O_RDONLY);
  if (fd_ < 0) {
    std::cerr << "Failed to open joystick device: " << device_path_ << std::endl;
    return false;
  }

  // Get joystick information
  char num_axes = 0;
  char num_buttons = 0;
  char name[256] = {0};
  
  if (ioctl(fd_, JSIOCGAXES, &num_axes) < 0 ||
      ioctl(fd_, JSIOCGBUTTONS, &num_buttons) < 0 ||
      ioctl(fd_, JSIOCGNAME(sizeof(name)), name) < 0) {
    std::cerr << "Failed to query joystick information" << std::endl;
    close();
    return false;
  }

  // Initialize state arrays
  axes_.resize(num_axes, 0.0f);
  buttons_.resize(num_buttons, 0);
  name_ = std::string(name);

  std::cout << "Opened joystick '" << name_ << "' with " 
            << static_cast<int>(num_axes) << " axes and " 
            << static_cast<int>(num_buttons) << " buttons" << std::endl;

  return true;
}

void JoystickInterface::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  axes_.clear();
  buttons_.clear();
  name_.clear();
}

int JoystickInterface::process_events(int timeout_ms)
{
  if (fd_ < 0) {
    return -1;
  }

  struct pollfd pfd;
  pfd.fd = fd_;
  pfd.events = POLLIN;
  pfd.revents = 0;

  int total_events = 0;
  int poll_result;
  
  // Poll for events with timeout
  while ((poll_result = poll(&pfd, 1, timeout_ms)) > 0) {
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      std::cerr << "Joystick poll error" << std::endl;
      return -1;
    }

    if (pfd.revents & POLLIN) {
      struct js_event event;
      if (read(fd_, &event, sizeof(event)) != sizeof(event)) {
        std::cerr << "Failed to read joystick event" << std::endl;
        return -1;
      }

      // Process the event
      switch (event.type & ~JS_EVENT_INIT) {
        case JS_EVENT_AXIS:
          if (event.number < axes_.size()) {
            axes_[event.number] = static_cast<float>(event.value) / MAX_AXIS_VALUE;
          } else {
            std::cerr << "Invalid axis number: " << event.number << std::endl;
          }
          break;
          
        case JS_EVENT_BUTTON:
          if (event.number < buttons_.size()) {
            buttons_[event.number] = event.value;
          } else {
            std::cerr << "Invalid button number: " << event.number << std::endl;
          }
          break;
      }
      
      total_events++;
    }
    
    // Set timeout to 0 for subsequent polls to process all available events
    timeout_ms = 0;
  }

  if (poll_result < 0) {
    std::cerr << "Joystick poll failed" << std::endl;
    return -1;
  }

  return total_events;
}

float JoystickInterface::get_axis(unsigned int idx) const
{
  if (idx >= axes_.size()) {
    std::cerr << "Invalid axis index: " << idx << std::endl;
    return 0.0f;
  }
  return axes_[idx];
}

int JoystickInterface::get_button(unsigned int idx) const
{
  if (idx >= buttons_.size()) {
    std::cerr << "Invalid button index: " << idx << std::endl;
    return 0;
  }
  return buttons_[idx];
}

void JoystickInterface::get_all_axes(std::vector<float> & axes) const
{
  axes = axes_;
}

void JoystickInterface::get_all_buttons(std::vector<int> & buttons) const
{
  buttons = buttons_;
}

}  // namespace joystick