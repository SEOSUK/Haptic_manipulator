/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

int main(int argc, char *argv[]) 
{
  const char* port_name = "/dev/ttyUSB0";
  int baud_rate = 57600;
  int dxl_id = 1;

  if (argc < 4)
  {
    printf("Please set '-port_name', '-baud_rate', '-dynamixel id' arguments for connected Dynamixels\n");
    return 0;
  }
  else
  {
    port_name = argv[1];
    baud_rate = atoi(argv[2]);
    dxl_id = atoi(argv[3]);
  }

  DynamixelWorkbench dxl_wb;

  const char *log;
  bool result = false;

  uint16_t model_number = 0;

  result = dxl_wb.init(port_name, baud_rate, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to init\n");

    return 0;
  }
  else
    printf("Succeed to init(%d)\n", baud_rate);  

  result = dxl_wb.ping(dxl_id, &model_number, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to ping\n");
  }
  else
  {
    printf("Succeed to ping\n");
    printf("id : %d, model_number : %d\n", dxl_id, model_number);
  }

  result = dxl_wb.itemWrite(dxl_id, "LED", 1, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to LED On\n");
  }
  else
  {
    printf("Succeed to LED On\n");
  }

  int32_t get_data = 0;
  result = dxl_wb.itemRead(dxl_id, "Position_P_Gain", &get_data, &log);
  if (result == false)
  {
    printf("%s\n", log);
    printf("Failed to get present position\n");
  }
  else
  {
    printf("Succeed to get Present Velocity(value : %d)\n", get_data);
  }

  // ADD
  dxl_wb.itemWrite(dxl_id, "Position_P_Gain", 500, &log);

  result = dxl_wb.itemRead(dxl_id, "Position_P_Gain", &get_data, &log);

  if(result == false)
  {
    printf("%s\n",log);
    printf("Failed to send position_p_gain\n");
  }
  else
  {
    printf("Succeed to send Position P Gain(value : %d)\n", get_data);
  }
  /* float get_data = 0;
  while(true)
  {
    result = dxl_wb.itemRead(dxl_id, "Present Current", &get_data, &log);
  
  if(result == false)
  {
    printf("Failed to read Present Current\n");
  }

  else
  {
    printf("present current : %lf\n", get_data);
  }

  } */
  return 0;
}
