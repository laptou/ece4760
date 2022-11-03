#ifndef _FPMATH_H
#define _FPMATH_H

typedef struct fixed {
  signed int value;
  
  constexpr static fixed from(float f) { return {(signed int)(f * 32768.0)}; }
  constexpr static fixed from(signed int i) { return {(i << 15)}; }
  constexpr static fixed zero() { return {0}; }

  fixed abs() const { return this->value > 0 ? *this : -*this; }

  constexpr fixed operator-() const { return {(-(this->value))}; }

  constexpr bool operator>(const fixed &rhs) const
  {
    return this->value > rhs.value;
  }

  constexpr bool operator<(const fixed &rhs) const
  {
    return this->value < rhs.value;
  }

  constexpr bool operator==(const fixed &rhs) const
  {
    return this->value == rhs.value;
  }

  constexpr bool operator>=(const fixed &rhs) const
  {
    return this->value >= rhs.value;
  }

  constexpr bool operator<=(const fixed &rhs) const
  {
    return this->value <= rhs.value;
  }

  constexpr fixed operator+(const fixed &rhs) const
  {
    return {this->value + rhs.value};
  }

  constexpr fixed operator-(const fixed &rhs) const
  {
    return {this->value - rhs.value};
  }

  constexpr fixed operator*(const fixed &rhs) const
  {
    auto lhs_v = (signed long long)(this->value);
    auto rhs_v = (signed long long)(rhs.value);
    return {(signed int)((lhs_v * rhs_v) >> 15)};
  }

  constexpr fixed operator/(const fixed &rhs) const
  {
    auto lhs_v = (signed long long)(this->value);
    auto rhs_v = (signed long long)(rhs.value);
    return {(signed int)((lhs_v << 15) / rhs_v)};
  }

  constexpr fixed operator>>(const int &rhs) const
  {
    return {(signed int)((this->value) >> rhs)};
  }

  constexpr fixed operator<<(const int &rhs) const
  {
    return {(signed int)((this->value) << rhs)};
  }

  constexpr operator float() const { return (float)this->value / 32768.0; }
  constexpr operator int() const { return (int)(this->value >> 15); }

  fixed &operator+=(const fixed &rhs)
  {
    this->value += rhs.value;
    return *this;
  }

  fixed &operator-=(const fixed &rhs)
  {
    this->value -= rhs.value;
    return *this;
  }

  fixed &operator*=(const fixed &rhs)
  {
    auto lhs_v = (signed long long)(this->value);
    auto rhs_v = (signed long long)(rhs.value);
    this->value = (signed int)((lhs_v * rhs_v) >> 15);
    return *this;
  }

  fixed &operator/=(const fixed &rhs)
  {
    auto lhs_v = (signed long long)(this->value);
    auto rhs_v = (signed long long)(rhs.value);
    this->value = (signed int)((lhs_v << 15) / rhs_v);
    return *this;
  }

  fixed &operator>>=(const int &rhs)
  {
    this->value >>= rhs;
    return *this;
  }

  fixed &operator<<=(const int &rhs)
  {
    this->value <<= rhs;
    return *this;
  }
} fixed;

#endif
