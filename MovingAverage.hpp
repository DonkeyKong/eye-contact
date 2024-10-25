#pragma once

template <typename T, int NumSamples = 3>
class MovingAverage
{
  T val_[NumSamples];
  int index_= -1;
public:
  void update(T val)
  {
    if (index_ == -1)
    {
      for (int i=0; i < NumSamples; ++i)
      {
        val_[i] = val;
      }
    }
    else
    {
      val_[index_] = val;
    }
    index_ = (index_ + 1) % NumSamples;
  }

  operator T() const
  {
    return val();
  }

  T val() const
  {
    double avg = 0;
    for (int i=0; i < NumSamples; ++i)
    {
      avg += (double)val_[i];
    }
    return (T) (avg / (double)NumSamples);
  }
};