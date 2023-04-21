/*
 * Copyright (C) 2021 Prensilia s.r.l.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors 
 *    may be used to endorse or promote products derived from this software 
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "mia_hand_driver/serial_port.h"

namespace mia_hand
{
FingerSerialInfo::FingerSerialInfo():
  mot_pos(0),
  mot_spe(0),
  mot_cur(0),
  fin_sg_raw{0, 0}
{

}

SerialPort::SerialPort(std::mutex* p_finger_data_mtx,
                       std::mutex* p_connection_mtx, bool* p_is_connected):
  p_finger_data_mtx_(p_finger_data_mtx),
  p_connection_mtx_(p_connection_mtx),
  p_is_connected_(p_is_connected)
{

}

SerialPort::~SerialPort()
{
  if (this->IsOpen())
  {
    this->Close();
  }
}

bool SerialPort::open(uint16_t port_num)
{
  if (this->IsOpen())
  {
    this->Close();
  }

  bool no_exceptions = true;

  std::string port_name = "/dev/ttyUSB";
  port_name += std::to_string(port_num);

  try
  {
    this->Open(port_name);
  }
  catch (LibSerial::OpenFailed& e)
  {
    no_exceptions = false;
  }
  catch (std::runtime_error& e)
  {
    no_exceptions = false;
  }

  if (no_exceptions)
  {
    this->SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    this->FlushIOBuffers();
  }

  return no_exceptions;
}

bool SerialPort::close()
{
  bool no_exceptions = true;

  try
  {
    this->Close();
  }
  catch (LibSerial::NotOpen& e)
  {
    // Keep program running.
  }
  catch (std::runtime_error& e)
  {
    no_exceptions = false;
  }

  return no_exceptions;
}

void SerialPort::sendCommand(const std::string& command)
{
  serial_write_mtx_.lock();

  try
  {
    Write(command);
    WriteByte((char) 0x2A);
    WriteByte((char) 0x0D);
  }
  catch (LibSerial::NotOpen& e)
  {
    // TODO: decide if notifying somehow the exception or not.
  }
  catch (std::runtime_error& e)
  {
    // TODO: decide if notifying somehow the exception or not.
  }

  serial_write_mtx_.unlock();

  return;
}

void SerialPort::parseStream(FingerSerialInfo& thumb,
                             FingerSerialInfo& index,
                             FingerSerialInfo& mrl,
                             bool& is_checking_on)
{
  bool no_exceptions = true;

  try
  {
    this->ReadLine(stream_msg_, '\n', 1000);
  }
  catch (LibSerial::NotOpen& e)
  {
    no_exceptions = false;
  }
  catch (std::runtime_error& e)
  {
    no_exceptions = false;

    p_connection_mtx_->lock();

    if (is_checking_on)
    {
      *p_is_connected_ = false;
    }

    p_connection_mtx_->unlock();
  }

  if (no_exceptions)
  {
    switch (stream_msg_[0])
    {
      case '<':

        if ('?' == stream_msg_[1])
        {
          p_connection_mtx_->lock();
          is_checking_on = false;
          *p_is_connected_ = true;
          p_connection_mtx_->unlock();
        }

      break;

      case 'e':

        p_finger_data_mtx_->lock();

        thumb.mot_pos = (stream_msg_[9]  - 48) * 100
                      + (stream_msg_[10] - 48) * 10 + stream_msg_[11] - 48;

        if ('-' == stream_msg_[6])
        {
          thumb.mot_pos = -thumb.mot_pos;
        }

        index.mot_pos = (stream_msg_[27] - 48) * 100
                      + (stream_msg_[28] - 48) * 10 + stream_msg_[29] - 48;

        if ('-' == stream_msg_[24])
        {
          index.mot_pos = -index.mot_pos;
        }

        mrl.mot_pos   = (stream_msg_[18] - 48) * 100
                      + (stream_msg_[19] - 48) * 10 + stream_msg_[20] - 48;

        if ('-' == stream_msg_[15])
        {
          mrl.mot_pos = -mrl.mot_pos;
        }

        p_finger_data_mtx_->unlock();

      break;

      case 's':

        p_finger_data_mtx_->lock();

        thumb.mot_spe = (stream_msg_[10] - 48) * 10 + stream_msg_[11] - 48;

        if ('-' == stream_msg_[6])
        {
          thumb.mot_spe = -thumb.mot_spe;
        }

        index.mot_spe = (stream_msg_[28] - 48) * 10 + stream_msg_[29] - 48;

        if ('-' == stream_msg_[24])
        {
          index.mot_spe = -index.mot_spe;
        }

        mrl.mot_spe   = (stream_msg_[19] - 48) * 10 + stream_msg_[20] - 48;

        if ('-' == stream_msg_[15])
        {
          mrl.mot_spe = -mrl.mot_spe;
        }

        p_finger_data_mtx_->unlock();

      break;

      case 'c':

        p_finger_data_mtx_->lock();

        thumb.mot_cur = (stream_msg_[8]  - 48) * 1000
                      + (stream_msg_[9]  - 48) * 100
                      + (stream_msg_[10] - 48) * 10 + stream_msg_[11] - 48;

        if ('-' == stream_msg_[6])
        {
          thumb.mot_cur = -thumb.mot_cur;
        }

        index.mot_cur = (stream_msg_[26] - 48) * 1000
                      + (stream_msg_[27] - 48) * 100
                      + (stream_msg_[28] - 48) * 10 + stream_msg_[29] - 48;

        if ('-' == stream_msg_[24])
        {
          index.mot_cur = -index.mot_cur;
        }

        mrl.mot_cur = (stream_msg_[17] - 48) * 1000
                    + (stream_msg_[18] - 48) * 100
                    + (stream_msg_[19] - 48) * 10 + stream_msg_[20] - 48;

        if ('-' == stream_msg_[15])
        {
          mrl.mot_cur = -mrl.mot_cur;
        }

        p_finger_data_mtx_->unlock();

      break;

      case 'a':

        p_finger_data_mtx_->lock();

        thumb.fin_sg_raw[0] = (stream_msg_[35] - 48) * 1000
                            + (stream_msg_[36] - 48) * 100
                            + (stream_msg_[37] - 48) * 10
                            + stream_msg_[38] - 48;

        if ('-' == stream_msg_[33])
        {
          thumb.fin_sg_raw[0] = -thumb.fin_sg_raw[0];
        }

        thumb.fin_sg_raw[1] = (stream_msg_[44] - 48) * 1000
                            + (stream_msg_[45] - 48) * 100
                            + (stream_msg_[46] - 48) * 10
                            + stream_msg_[47] - 48;

        if ('-' == stream_msg_[42])
        {
          thumb.fin_sg_raw[1] = -thumb.fin_sg_raw[1];
        }

        index.fin_sg_raw[0] = (stream_msg_[26] - 48) * 1000
                            + (stream_msg_[27] - 48) * 100
                            + (stream_msg_[28] - 48) * 10
                            + stream_msg_[29] - 48;

        if ('-' == stream_msg_[24])
        {
          index.fin_sg_raw[0] = -index.fin_sg_raw[0];
        }

        index.fin_sg_raw[1] = (stream_msg_[17] - 48) * 1000
                            + (stream_msg_[18] - 48) * 100
                            + (stream_msg_[19] - 48) * 10
                            + stream_msg_[20] - 48;

        if ('-' == stream_msg_[15])
        {
          index.fin_sg_raw[1] = -index.fin_sg_raw[1];
        }

        mrl.fin_sg_raw[0]   = (stream_msg_[8]  - 48) * 1000
                            + (stream_msg_[9]  - 48) * 100
                            + (stream_msg_[10] - 48) * 10
                            + stream_msg_[11] - 48;

        if ('-' == stream_msg_[6])
        {
          mrl.fin_sg_raw[0] = -mrl.fin_sg_raw[0];
        }

        mrl.fin_sg_raw[1]   = (stream_msg_[53] - 48) * 1000
                            + (stream_msg_[54] - 48) * 100
                            + (stream_msg_[55] - 48) * 10
                            + stream_msg_[56] - 48;

        if ('-' == stream_msg_[51])
        {
          mrl.fin_sg_raw[1] = -mrl.fin_sg_raw[1];
        }

        p_finger_data_mtx_->unlock();

      break;

      default:

      break;
    }
  }

  return;
}
}  // namespace
