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

#include "fkie_iop_component/timer.hpp"

using namespace iop;

Timer::Timer(const std::chrono::milliseconds &interval,
             const std::function<void ()> &task,
             bool single_shot)
    : m_interval(interval),
      m_task(task),
      m_single_shot(single_shot),
      m_running(false)
{
}

Timer::~Timer()
{
    stop();
}

bool Timer::is_running() const
{
    return m_running;
}

void Timer::start()
{
    stop();

    m_running = true;
    m_thread = std::thread([this]
    {
        while (m_running)
        {
            std::unique_lock<std::mutex> lock(m_mutext_run_cond);
            auto waitResult = m_run_condition.wait_for(lock, m_interval, [this]{ return !m_running; });
            if (m_running && !waitResult)
                m_task();

            if(m_single_shot)
                m_running = false;
        }
    });
}

void Timer::set_interval(const std::chrono::milliseconds &interval)
{
    m_interval = interval;
}

void Timer::set_rate(double hz)
{
    m_interval = std::chrono::milliseconds((int)(1000/hz));
}

void Timer::set_single_shot(bool enabled)
{
    m_single_shot = enabled;
}

void Timer::stop()
{
    std::unique_lock<std::mutex> lock(m_mutex_stop);
    m_running = false;
    m_run_condition.notify_all();
    if(m_thread.joinable())
        m_thread.join();
}