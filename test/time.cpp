#include <stdio.h>
#include <unistd.h>
#include <time.h>
void cur_time(void);
char *wday[]={"Sunday","Monday","Tuesday","Wednesday","Thursday","Friday","Saturday"};
void realTime(unsigned int &year, 
              unsigned int &mon, 
              unsigned int &day, 
              unsigned int &wday, 
              unsigned int &hour, 
              unsigned int &min, 
              unsigned int &sec);
int main(int argc,char **argv)
{
  while(1)
  {
    unsigned int year, mon, wday, day, hour, min, sec;
    realTime(year, mon, day, wday, hour, min, sec);
    printf("%d year %02d month %02d day",year,mon,day);
    printf("%02d:%02d:%02d -- %d\n",hour,min,sec,hour*60+min);
    printf("\n");
    sleep(1);
  }
  return 0;
}

void realTime(unsigned int &year, 
              unsigned int &mon, 
              unsigned int &day, 
              unsigned int &wday, 
              unsigned int &hour, 
              unsigned int &min, 
              unsigned int &sec)
{
  time_t timep;
  struct tm *p;
  time(&timep);
  p=localtime(&timep); /* 获取当前时间 */
  year = 1900+p->tm_year;
  mon  = 1+p->tm_mon;
  day  = p->tm_mday;
  wday = p->tm_wday;
  hour = p->tm_hour;
  min  = p->tm_min;
  sec  = p->tm_sec;
}
