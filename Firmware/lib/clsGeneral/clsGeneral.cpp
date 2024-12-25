
#include "clsGeneral.h"

StorageDataLong::StorageDataLong(String ID, String StorageID, String Description, long Value, long MinValue, long MaxValue, long Step, String Unit)
{
  this->StorageID = StorageID;
  this->Description = Description;
  this->Value = Value;
  this->MinValue = MinValue;
  this->MaxValue = MaxValue;
  this->Step = Step;
  this->Unit = Unit;
}
long StorageDataLong::GetValue()
{
  return Value * Step;
}
void StorageDataLong::SetValue(long Value)
{
  this->Value = Value / Step;
}
float MapValue(float x, float in_min, float in_max, float out_min, float out_max)
{
  if ((in_max == in_min) || (out_max == out_min))
    return out_max;
  else
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void Swap(long *Long1, long *Long2)
{
  long *tmp;
  *tmp = *Long1;
  *Long1 = *Long2;
  *Long2 = *tmp;
}
void AdjustRange(long *Min, long *Max)
{
  if (*Min > *Max)
    Swap(Min, Max);
}
long AdjustValue(long Min, long Max, long Value)
{
  AdjustRange(&Min, &Max);
  if (Value < Min)
    return Min;
  else if (Value > Max)
    return Max;
  else
    return Value;
}
bool ValueInRange(long Min, long Max, long Value)
{
  AdjustRange(&Min, &Max);
  return ((Value >= Min) && (Value <= Max));
}
long MaxValue(long Value1, long Value2)
{
  if (Value1 > Value2)
    return Value1;
  else
    return Value2;
}