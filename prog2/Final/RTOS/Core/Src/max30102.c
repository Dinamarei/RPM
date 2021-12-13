#include "stm32l4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

int heart_rate_array[4];
int previous_reading = 0;
int heart_rate = 0;
int ir[15] = {0};
uint8_t reg_data;
uint8_t data;
uint32_t read_fifo_red, read_fifo_ir;
uint8_t dat[6] = {0,0,0,0,0,0};
uint32_t read_seq_red[100];
int read_seq_ir[100];
uint8_t n=100;

int findsamples(int h[], int size, int t, int max)
{
    int i = 0;
    int peaks = 0;
    while (i < size - 1)
    {
        if (h[i] > t && h[i] > h[i - 1])
        {
            int width = 1;
            while (i + width < size - 1 && h[i] == h[i + width]) 
                width++;
            if (h[i] > h[i + width] && peaks < max) 
            {
                ir[peaks] = i;
                int k = peaks - 1;
                while (k >= 0 && i > ir[k]) 
                {
                    int temp = ir[k];
                    ir[k] = i;						
                    ir[k + 1] = temp;
                    k--;
                }
                peaks++; 
                i += width + 1; 
            }
            else
                i += width;		
        }
        else
            i++;
    }
    return peaks;
}
int remove_fluct(int peaks, int rd[], int min)
{
    int i = -1;
    while (i < peaks)
    {
        int old_n_peaks = peaks;    	
        peaks = i + 1;
        int j = i + 1;
        while (j < old_n_peaks) 
        {
            int n_dist = i != -1 ? (ir[j] - ir[i]) : (ir[j] + 1);
            if (n_dist > min || n_dist < -1 * min)
            {
                ir[peaks] = ir[j];
                peaks++;										
            }
            j++;
        }
        i++;
    }
    for (int i = 0; i < peaks - 1; i++)			
        for (int j = i + 1; j < peaks; j++)
            if (ir[i] > ir[j])
            {
                int temp = ir[i];				
                ir[i] = ir[j]; 
                ir[j] = temp;						
            }
}
int calculate(int rd[])
{
    int sum = 0;
    for (int i = 0; i < n; i++) 
        sum += rd[i];
    int mean = sum / n;
    for (int i = 0; i < n; i++) 
        rd[i] = (rd[i] - mean) * -1;
    for (int i = 0; i < n-4; i++)
        rd[i] = (rd[i] + rd[i + 1] + rd[i + 2] + rd[i + 3]) / 4; 
    sum = 0;
    for (int i = 0; i < n; i++)
        sum += rd[i]; 
    mean = sum / n;
    int t = mean < 30 ? 30 : mean > 60 ? 60 : mean;
    int peaks = findsamples(rd, 100, t, 15); 
    int peaks2 = remove_fluct(peaks, rd, 4); 
    peaks = peaks < peaks2 ? peaks : peaks2; 
    int peaks_interval = 0;
    if (peaks >= 2) 
    {
        for (int i = 1; i < peaks; i++)
            peaks_interval += (ir[i] - ir[i - 1]); 
            peaks_interval = peaks_interval / (peaks - 1);
            heart_rate = 25 * 60 / peaks_interval; 
			previous_reading = heart_rate>30 && heart_rate<170?heart_rate:previous_reading; 
			heart_rate = heart_rate>30 && heart_rate<170?heart_rate:-999;
    }
    else
        heart_rate = -999;
    return heart_rate;
}
void init(){

	    if(HAL_I2C_IsDeviceReady(&hi2c1, 0xAE, 1, 100)!=HAL_OK){HAL_Delay(1000);}
				
        HAL_I2C_Mem_Write(&hi2c1, 0xAE, 0x09, I2C_MEMADD_SIZE_8BIT, &data , 1, 10);	

        HAL_Delay(10);
        
        HAL_I2C_Mem_Read(&hi2c1, 0xAF, 0x00, I2C_MEMADD_SIZE_8BIT, &reg_data, 1, 100);
        data = 0xc0;
        HAL_I2C_Mem_Write(&hi2c1, 0xAE, 0x02, I2C_MEMADD_SIZE_8BIT, &data , 1, 10);
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, 0xAE, 0x03, I2C_MEMADD_SIZE_8BIT, &data , 1, 10);
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, 0xAE, 0x04, I2C_MEMADD_SIZE_8BIT, &data , 1, 10);
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, 0xAE, 0x05, I2C_MEMADD_SIZE_8BIT, &data , 1, 10);
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, 0xAE, 0x06, I2C_MEMADD_SIZE_8BIT, &data , 1, 10);
        data = 0x4f;
        HAL_I2C_Mem_Write(&hi2c1, 0xAE, 0x08, I2C_MEMADD_SIZE_8BIT, &data , 1, 10);
        data = 0x03;
        HAL_I2C_Mem_Write(&hi2c1, 0xAE, 0x09, I2C_MEMADD_SIZE_8BIT, &data , 1, 10);
        data = 0x27;
        HAL_I2C_Mem_Write(&hi2c1, 0xAE, 0x0A, I2C_MEMADD_SIZE_8BIT, &data , 1, 10);
        data = 0x24;
        HAL_I2C_Mem_Write(&hi2c1, 0xAE, 0x0C, I2C_MEMADD_SIZE_8BIT, &data , 1, 10);
        data = 0x24;
        HAL_I2C_Mem_Write(&hi2c1, 0xAE, 0x0D, I2C_MEMADD_SIZE_8BIT, &data , 1, 10);
        data = 0x7f;
        HAL_I2C_Mem_Write(&hi2c1, 0xAE, 0x10, I2C_MEMADD_SIZE_8BIT, &data , 1, 10);
}

void read_fifo(){
    uint8_t reg_INTR1, reg_INTR2;
    HAL_I2C_Mem_Read(&hi2c1, 0xAE, 0x00, I2C_MEMADD_SIZE_8BIT, &reg_INTR1, 1, 10);
    HAL_I2C_Mem_Read(&hi2c1, 0xAE, 0x01, I2C_MEMADD_SIZE_8BIT, &reg_INTR2, 1, 10);
    HAL_I2C_Mem_Read(&hi2c1, 0xAE, 0x07, I2C_MEMADD_SIZE_8BIT, dat, 6 , 25);
    read_fifo_red = (dat[0] << 16 | dat[1] << 8 | dat[2]) & 0x03FFFF;
    read_fifo_ir = (dat[3] << 16 | dat[4] << 8 | dat[5]) & 0x03FFFF;
}

int read_sequential(){
    int i;
		int c999 = 0;
		for(int ii = 0; ii<4; ii++){
    for(i=0;i<n;i++){
        while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 1){} 
        read_fifo();
        read_seq_red[i] = read_fifo_red; 
        read_seq_ir[i] = read_fifo_ir;
    }
		heart_rate_array[ii] = calculate(read_seq_ir);
		if(heart_rate_array[ii] == -999)
			c999++;
	}
		return (heart_rate_array[0]+heart_rate_array[1]+heart_rate_array[2]+heart_rate_array[3]+(999*c999))/(4-c999); 
}