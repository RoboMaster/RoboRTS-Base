/****************************************************************************
 *  Copyright (C) 2021 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef ROBORTS_BASE_UTILS_BIND_THIS_H
#define ROBORTS_BASE_UTILS_BIND_THIS_H

namespace std
{
template<int>  // begin with 0 here!
struct placeholder_template
{};

template<int N>
struct is_placeholder<placeholder_template<N> >
    : integral_constant<int, N+1>  // the one is important
{};
}  // end of namespace std


template<int...>
struct int_sequence
{};

template<int N, int... Is>
struct make_int_sequence
    : make_int_sequence<N-1, N-1, Is...>
{};

template<int... Is>
struct make_int_sequence<0, Is...>
    : int_sequence<Is...>
{};

template<class R, class U, class... Args, int... Is>
auto bind_this_sub(R (U::*p)(Args...), U * pp, int_sequence<Is...>)
-> decltype(std::bind(p, pp, std::placeholder_template<Is>{}...))
{
  return std::bind(p, pp, std::placeholder_template<Is>{}...);
}

// binds a member function only for the this pointer using std::bind
template<class R, class U, class... Args>
auto bind_this(R (U::*p)(Args...), U * pp)
-> decltype(bind_this_sub(p, pp, make_int_sequence< sizeof...(Args) >{}))
{
  return bind_this_sub(p, pp, make_int_sequence< sizeof...(Args) >{});
}

// utility
template<class R, class U, class... Args, int... Is>
auto bind_this_sub(R (U::*p)(Args...) const, U * pp, int_sequence<Is...>)
-> decltype(std::bind(p, pp, std::placeholder_template<Is>{}...))
{
  return std::bind(p, pp, std::placeholder_template<Is>{}...);
}

// binds a member function only for the this pointer using std::bind
template<class R, class U, class... Args>
auto bind_this(R (U::*p)(Args...) const, U * pp)
-> decltype(bind_this_sub(p, pp, make_int_sequence< sizeof...(Args) >{}))
{
  return bind_this_sub(p, pp, make_int_sequence< sizeof...(Args) >{});
}

#endif //ROBORTS_BASE_UTILS_BIND_THIS_H
