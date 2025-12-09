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

#ifndef JOYSTICK__JOYSTICK_INTERFACE_HPP_
#define JOYSTICK__JOYSTICK_INTERFACE_HPP_

#include <string>
#include <vector>

namespace joystick
{

class JoystickInterface
{
public:
  explicit JoystickInterface(const std::string & device_path = "/dev/input/js0");
  ~JoystickInterface();

  bool open();
  void close();
  bool is_open() const { return fd_ >= 0; }
  
  std::string get_name() const { return name_; }
  
  // Process joystick events (non-blocking)
  // Returns number of events processed, -1 on error
  int process_events(int timeout_ms = 0);
  
  // Get current axis value (normalized to [-1.0, 1.0])
  float get_axis(unsigned int idx) const;
  
  // Get current button state (0 or 1)
  int get_button(unsigned int idx) const;
  
  // Get all current states
  void get_all_axes(std::vector<float> & axes) const;
  void get_all_buttons(std::vector<int> & buttons) const;
  
  size_t get_num_axes() const { return axes_.size(); }
  size_t get_num_buttons() const { return buttons_.size(); }

private:
  int fd_;
  std::string device_path_;
  std::string name_;
  std::vector<float> axes_;
  std::vector<int> buttons_;
  
  static constexpr int MAX_AXIS_VALUE = 32767;
};

}  // namespace joystick

#endif  // JOYSTICK__JOYSTICK_INTERFACE_HPP_