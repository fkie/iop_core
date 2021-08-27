/**
ROS/IOP Bridge
Copyright (c) 2017 Fraunhofer

This program is dual licensed; you can redistribute it and/or
modify it under the terms of the GNU General Public License
version 2 as published by the Free Software Foundation, or
enter into a proprietary license agreement with the copyright
holder.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; or you can read the full license at
<http://www.gnu.de/documents/gpl-2.0.html>
*/

/** \author Alexander Tiderko */

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <thread>

namespace iop {

class Timer
{
  public:
    Timer(const std::chrono::milliseconds &interval,
          const std::function<void ()> &task,
          bool single_shot=false);
    Timer(const std::chrono::seconds &interval,
          const std::function<void ()> &task,
          bool single_shot=false);
    ~Timer();

    void start();
    void stop();
    void set_interval(const std::chrono::seconds &interval);
    void set_interval(const std::chrono::milliseconds &interval);
    std::chrono::milliseconds get_interval();
    void set_rate(double hz);
    void set_single_shot(bool enabled = true);
    bool is_running() const;

  private:
    uint64_t m_current_id;  // Control variable to ensure that the timer has expired in the correct thread after the restart
    std::chrono::milliseconds m_interval;
    std::function<void ()> m_task;
    std::atomic_bool m_single_shot;
    std::atomic_bool m_running;
    std::condition_variable m_run_condition;
    std::mutex m_mutext_run_cond;
    std::thread m_thread;
    std::mutex m_mutex_stop;

    void m_timeout(uint64_t id);

};

}