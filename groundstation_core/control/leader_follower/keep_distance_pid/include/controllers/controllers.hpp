////////////////////////////////////////////////////////////////////////////////
//******************************************************************************************************* 
//*******This composable node is responsible to force followers to follow thier leader ********
//******************************************************************************************************* 
// Copyright (c) 2025/1 NTU UAV lab Inc.
// Auther :Morteza Aliyari
// Email: mortezaliyari@gmail.com
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////

#pragma once
#include <stdint.h>

#include <chrono>
#include <iostream>
using namespace std::chrono;
using namespace std::chrono_literals;
namespace Controllers{
    class PID
    {
        public:
        PID(double kp, double ki, double kd);
        ~PID();
        double compute(double desire_input,double measured_feedback,double dt);
        void set_coefficients(double kp, double ki, double kd);
        void print_coefficients();

        private:
        // PID coefficients
        double kp_;
        double ki_;
        double kd_;
        // State variables
        double prev_error_;
        double integral_;

    };
}
