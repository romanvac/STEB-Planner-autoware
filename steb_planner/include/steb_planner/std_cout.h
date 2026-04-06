//
// Created by hs on 24-3-20.
//
/*********************************************************************
 *
 * Software License Agreement (MIT License)
 *
 * Copyright (c) 2024, HeShan.
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES, OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT, OR OTHERWISE, ARISING FROM, OUT OF, OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Author: HeShan
 *********************************************************************/

#ifndef STD_COUT_H_
#define STD_COUT_H_

//
//  main.cpp
//  ColoredHelloWorld
//
//  Created by obaby on 14-2-27.
//  Copyright (c) 2014年 mars. All rights reserved.
//

#include <iostream>

// the following are UBUNTU/LINUX ONLY terminal color codes.
#define OUT_RESET "\033[0m"
#define OUT_BLACK "\033[30m"              /* Black */
#define OUT_RED "\033[31m"                /* Red */
#define OUT_GREEN "\033[32m"              /* Green */
#define OUT_YELLOW "\033[33m"             /* Yellow */
#define OUT_BLUE "\033[34m"               /* Blue */
#define OUT_MAGENTA "\033[35m"            /* Magenta */
#define OUT_CYAN "\033[36m"               /* Cyan */
#define OUT_WHITE "\033[37m"              /* White */
#define OUT_BOLDBLACK "\033[1m\033[30m"   /* Bold Black */
#define OUT_BOLDRED "\033[1m\033[31m"     /* Bold Red */
#define OUT_BOLDGREEN "\033[1m\033[32m"   /* Bold Green */
#define OUT_BOLDYELLOW "\033[1m\033[33m"  /* Bold Yellow */
#define OUT_BOLDBLUE "\033[1m\033[34m"    /* Bold Blue */
#define OUT_BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define OUT_BOLDCYAN "\033[1m\033[36m"    /* Bold Cyan */
#define OUT_BOLDWHITE "\033[1m\033[37m"   /* Bold White */

// example
// std::cout<<OUT_RED      <<"Hello, World! in RED"       << OUT_RESET << std::endl;
// std::cout<<OUT_GREEN    <<"Hello, World! in GREEN"     << OUT_RESET << std::endl;
// std::cout<<OUT_YELLOW   <<"Hello, World! in YELLOW"    << OUT_RESET << std::endl;

#endif  // STD_COUT_H_
