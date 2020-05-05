#ifndef INFINITE_THREAD_H
#define INFINITE_THREAD_H

#include <QObject>
#include <QDialog>
#include <QSerialPort>
#include <QByteArray>
#include <slalib.h>
#include <slamac.h>

class InfiniteCountWorker : public QObject
{
    Q_OBJECT

    public:
        InfiniteCountWorker(double, double);
        void Coord(double,double);
        QByteArray readSerialAzimuthEncoder();
        void sendSerialData(QByteArray);
        void readSerialAzimuthEncoder2();
        void sendSerialData2(QByteArray);
        void PID_loop();

        void pwmOut(int);






    public slots:
        void doWork();
        void stopWork();
   signals:

        void updateInfiniteCount(double,int,int,int,int,double,double,char,char,int,int,int,int,int,int,char,char,int,int,int,int,int,int,QString,QString);
        void finished();




    private:
        QSerialPort *serial;
        QByteArray serialData;
        QString serialBuffer;
        QByteArray parsed_data;
        QByteArray serialData2;
        QString serialBuffer2;
        QByteArray parsed_data2;
        QString filename;
        QString filename2;
        QString current_time;
        QString current_date;

            bool m_running;
            int azd,azm,altd,altm;
            double asec,altsec;
            double ams,converted_ra,converted_dec;
            char signRA, signDEC;
            int chour,cminute,csecond,cdeg,camin,casec;
            double rr,dd;

            char signha, signde;
            int idmsfdec[4];
            int ihmsfha[4];

            int angle;
            int encoderValue;
            int REV; //set point


};
#endif // INFINITE_THREAD_H
