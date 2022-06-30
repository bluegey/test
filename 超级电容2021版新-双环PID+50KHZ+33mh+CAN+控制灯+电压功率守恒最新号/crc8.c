#include "stdio.h"

//通过反序
unsigned char cal_table_low_first(unsigned char value)
{
    unsigned char i, crc;

    crc = value;
    for (i = 8; i > 0; --i)
    {
        if (crc & 0x01)
        {
            crc = (crc >> 1) ^ 0x8C;//将0x31进行反序
        }
        else
            crc = (crc >> 1);
    }

    return crc;
}

void  create_crc_table(void)
{
    unsigned short i;
    unsigned char j;

    for (i = 0; i <= 0xFF; i++)
    {
        if (0 == (i % 16))
            printf("\n");

        j = i & 0xFF;
        printf("0x%.2x, ", cal_table_low_first(j));
    }
}
int main(void)
{
    create_crc_table();
}