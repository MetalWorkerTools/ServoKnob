#pragma once
#include <Arduino.h>

class StorageDataLong
{
private:
    /* data */
public:
    String ID="ID";
    String StorageID = "StorageID";
    String Description = "Description";
    long Value = 0;
    long MinValue = 0;
    long MaxValue = 0;
    long Step = 1;
    String Unit = "Unit";

    StorageDataLong(String ID, String StorageID, String Description, long Value, long MinValue, long MaxValue, long Step, String Unit);
    long GetValue();
    void SetValue(long Value);
};

float MapValue(float x, float in_min, float in_max, float out_min, float out_max);
long AdjustValue(long Min, long Max, long Value);
bool ValueInRange( long Min, long Max,long Value);
long MaxValue( long Value1, long Value2);
