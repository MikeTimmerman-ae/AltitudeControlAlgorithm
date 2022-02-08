/**
 *	\file header.h
 *	\author Mike Timmerman
 *	\version 3.0
 *	\date 2022
 */

#pragma once

namespace P                         // namespace parameters
{
    const double dt = 0.01;
    const int n_states = 2;
    const int n_control_input = 1;
    const int n_output = 1;
    const int n_rbf = 100;
    const int n_states_lift = 102;
    const int t_prediction = 1.0;
    const int n_prediction = 100;      
}
