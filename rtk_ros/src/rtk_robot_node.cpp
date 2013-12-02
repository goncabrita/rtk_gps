/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Goncalo Cabrita and Jorge Fraga on 13/06/2013
*********************************************************************/

#include <rtklib/rtklib.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/ByteMultiArray.h>
#include <rtk_msgs/Status.h>
#include <rtk_msgs/ECEFCoordinates.h>

#define BUFFSIZE 32768

#define RTK_ROBOT           0
#define RTK_BASE_STATION 	1

#define DLL

rtksvr_t server;

rtk_msgs::ECEFCoordinates ecef_base_station;

void baseStationCallback(const std_msgs::ByteMultiArray::ConstPtr& msg)
{
    int n = msg->data.size();
	
    unsigned char *p = server.buff[RTK_BASE_STATION]+server.nb[RTK_BASE_STATION];
    unsigned char *q = server.buff[RTK_BASE_STATION]+server.buffsize;

    if(server.nb[RTK_BASE_STATION] + n < server.buffsize)
    {
	for(int i=0 ; i<n ; i++)
	{
	    *(server.buff[RTK_BASE_STATION] + server.nb[RTK_BASE_STATION] + i) = msg->data[i];
	}
	server.nb[RTK_BASE_STATION] += n;

	/* write receiver raw/rtcm data to log stream */
        strwrite(server.stream+6, p, n);
        server.nb[RTK_BASE_STATION] += n;
            
        /* save peek buffer */
        rtksvrlock(&server);
        n = n < server.buffsize - server.npb[RTK_BASE_STATION] ? n : server.buffsize - server.npb[RTK_BASE_STATION];
        memcpy(server.pbuf[RTK_BASE_STATION] + server.npb[RTK_BASE_STATION], p, n);
        server.npb[RTK_BASE_STATION] += n;
        rtksvrunlock(&server);
    }
}

void ecefCallback(const rtk_msgs::ECEFCoordinates::ConstPtr& msg)
{
    ecef_base_station = *msg;
    
    server.rtk.opt.rb[0] = msg->position.x;
    server.rtk.opt.rb[1] = msg->position.y;
    server.rtk.opt.rb[2] = msg->position.z;
}

/* update navigation data ----------------------------------------------------*/
static void updatenav(nav_t *nav)
{
    int i,j;
    for (i=0;i<MAXSAT;i++) for (j=0;j<NFREQ;j++) {
        nav->lam[i][j]=satwavelen(i+1,j,nav);
    }
}

/* update rtk server struct --------------------------------------------------*/
static void updatesvr(rtksvr_t *svr, int ret, obs_t *obs, nav_t *nav, int sat,
                      sbsmsg_t *sbsmsg, int index, int iobs)
{
    eph_t *eph1,*eph2,*eph3;
    geph_t *geph1,*geph2,*geph3;
    gtime_t tof;
    double pos[3],del[3]={0},dr[3];
    int i,n=0,prn,sbssat=svr->rtk.opt.sbassatsel;
    
    if (ret==1) { /* observation data */
        if (iobs<MAXOBSBUF) {
            for (i=0;i<obs->n;i++) {
                if (svr->rtk.opt.exsats[obs->data[i].sat-1]==1||
                    !(satsys(obs->data[i].sat,NULL)&svr->rtk.opt.navsys)) continue;
                svr->obs[index][iobs].data[n]=obs->data[i];
                svr->obs[index][iobs].data[n++].rcv=index+1;
            }
            svr->obs[index][iobs].n=n;
            sortobs(&svr->obs[index][iobs]);
        }
        svr->nmsg[index][0]++;
    }
    else if (ret==2) { /* ephemeris */
        if (satsys(sat,&prn)!=SYS_GLO) {
            if (!svr->navsel||svr->navsel==index+1) {
                eph1=nav->eph+sat-1;
                eph2=svr->nav.eph+sat-1;
                eph3=svr->nav.eph+sat-1+MAXSAT;
                if (eph2->ttr.time==0||
                    (timediff(eph1->toe,eph2->toe)>0.0&&eph1->iode!=eph2->iode)) {
                    *eph3=*eph2;
                    *eph2=*eph1;
                    updatenav(&svr->nav);
                }
            }
            svr->nmsg[index][1]++;
        }
        else {
           if (!svr->navsel||svr->navsel==index+1) {
               geph1=nav->geph+prn-1;
               geph2=svr->nav.geph+prn-1;
               geph3=svr->nav.geph+prn-1+MAXPRNGLO;
               if (geph2->tof.time==0||
                   (timediff(geph1->toe,geph2->toe)>0.0&&geph1->iode!=geph2->iode)) {
                   *geph3=*geph2;
                   *geph2=*geph1;
                   updatenav(&svr->nav);
               }
           }
           svr->nmsg[index][6]++;
        }
    }
    else if (ret==3) { /* sbas message */
        if (sbsmsg&&(sbssat==sbsmsg->prn||sbssat==0)) {
            if (svr->nsbs<MAXSBSMSG) {
                svr->sbsmsg[svr->nsbs++]=*sbsmsg;
            }
            else {
                for (i=0;i<MAXSBSMSG-1;i++) svr->sbsmsg[i]=svr->sbsmsg[i+1];
                svr->sbsmsg[i]=*sbsmsg;
            }
            sbsupdatecorr(sbsmsg,&svr->nav);
        }
        svr->nmsg[index][3]++;
    }
    else if (ret==9) { /* ion/utc parameters */
        if (svr->navsel==index||svr->navsel>=3) {
            for (i=0;i<8;i++) svr->nav.ion_gps[i]=nav->ion_gps[i];
            for (i=0;i<4;i++) svr->nav.utc_gps[i]=nav->utc_gps[i];
            for (i=0;i<4;i++) svr->nav.ion_gal[i]=nav->ion_gal[i];
            for (i=0;i<4;i++) svr->nav.utc_gal[i]=nav->utc_gal[i];
            for (i=0;i<8;i++) svr->nav.ion_qzs[i]=nav->ion_qzs[i];
            for (i=0;i<4;i++) svr->nav.utc_qzs[i]=nav->utc_qzs[i];
            svr->nav.leaps=nav->leaps;
        }
        svr->nmsg[index][2]++;
    }
    else if (ret==5) { /* antenna postion parameters */
        if (svr->rtk.opt.refpos==4&&index==1) {
            for (i=0;i<3;i++) {
                svr->rtk.rb[i]=svr->rtcm[1].sta.pos[i];
            }
            /* antenna delta */
            ecef2pos(svr->rtk.rb,pos);
            if (svr->rtcm[1].sta.deltype) { /* xyz */
                del[2]=svr->rtcm[1].sta.hgt;
                enu2ecef(pos,del,dr);
                for (i=0;i<3;i++) {
                    svr->rtk.rb[i]+=svr->rtcm[1].sta.del[i]+dr[i];
                }
            }
            else { /* enu */
                enu2ecef(pos,svr->rtcm[1].sta.del,dr);
                for (i=0;i<3;i++) {
                    svr->rtk.rb[i]+=dr[i];
                }
            }
        }
        svr->nmsg[index][4]++;
    }
    else if (ret==7) { /* dgps correction */
        svr->nmsg[index][5]++;
    }
    else if (ret==10) { /* ssr message */
        for (i=0;i<MAXSAT;i++) {
            if (!svr->rtcm[index].ssr[i].update) continue;
            svr->rtcm[index].ssr[i].update=0;
            svr->nav.ssr[i]=svr->rtcm[index].ssr[i];
        }
        svr->nmsg[index][7]++;
    }
    else if (ret==31) { /* lex message */
        lexupdatecorr(&svr->raw[index].lexmsg,&svr->nav,&tof);
        svr->nmsg[index][8]++;
    }
    else if (ret==-1) { /* error */
        svr->nmsg[index][9]++;
    }
}

/* decode receiver raw/rtcm data ---------------------------------------------*/
static int decoderaw(rtksvr_t *svr, int index)
{
    obs_t *obs;
    nav_t *nav;
    sbsmsg_t *sbsmsg=NULL;

    int i,ret,sat,fobs=0;
    
    tracet(4,"decoderaw: index=%d\n",index);
    
    rtksvrlock(svr);
    
    for (i=0;i<svr->nb[index];i++) {
        
        /* input rtcm/receiver raw data from stream */
        if (svr->format[index]==STRFMT_RTCM2) {
            ret=input_rtcm2(svr->rtcm+index,svr->buff[index][i]);
            obs=&svr->rtcm[index].obs;
            nav=&svr->rtcm[index].nav;
            sat=svr->rtcm[index].ephsat;
        }
        else if (svr->format[index]==STRFMT_RTCM3) {
            ret=input_rtcm3(svr->rtcm+index,svr->buff[index][i]);
            obs=&svr->rtcm[index].obs;
            nav=&svr->rtcm[index].nav;
            sat=svr->rtcm[index].ephsat;
        }
        else {
            ret=input_raw(svr->raw+index,svr->format[index],svr->buff[index][i]);

            obs=&svr->raw[index].obs;
            nav=&svr->raw[index].nav;
            sat=svr->raw[index].ephsat;
            sbsmsg=&svr->raw[index].sbsmsg;
        }
        /* update rtk server */
        if (ret>0) updatesvr(svr,ret,obs,nav,sat,sbsmsg,index,fobs);
        
        /* observation data received */
        if(ret==1) {
            if (fobs<MAXOBSBUF) fobs++; else svr->prcout++;
        }
    }
    svr->nb[index]=0;
    
    rtksvrunlock(svr);
    
    return fobs;
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "rtk_robot");

    ROS_INFO("RTKlib for ROS Robot Edition");

    ros::NodeHandle nn;
    ros::NodeHandle pn("~");

    ros::Subscriber ecef_sub;
    if(pn.getParam("base_position/x", ecef_base_station.position.x) && pn.getParam("base_position/y", ecef_base_station.position.y) && pn.getParam("base_position/z", ecef_base_station.position.z))
    {
        ROS_INFO("RTK -- Loading base station parameters from the parameter server...");

        XmlRpc::XmlRpcValue position_covariance;
        if( pn.getParam("base_position/covariance", position_covariance) )
        {
            ROS_ASSERT(position_covariance.getType() == XmlRpc::XmlRpcValue::TypeArray);

            if(position_covariance.size() != 9)
            {
                ROS_WARN("RTK -- The base station covariances are not complete! Using default values...");
            }
            else
            {
                for(int i=0 ; i<position_covariance.size() ; ++i)
                {
                    ROS_ASSERT(position_covariance[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

                    ecef_base_station.position_covariance[i] = static_cast<double>(position_covariance[i]);
                }
            }
        }
    }
    else
    {
        ROS_INFO("RTK -- Subscribing to the base station for online parameters...");

        ecef_sub = nn.subscribe("base_station/gps/ecef", 50, ecefCallback);
    }

    double rate;
    pn.param("rate", rate, 2.0);

    std::string gps_frame_id;
    pn.param<std::string>("gps_frame_id", gps_frame_id, "gps");

    std::string port;
    pn.param<std::string>("port", port, "ttyACM0");
    int baudrate;
    pn.param("baudrate", baudrate, 115200);

    ros::Publisher gps_pub = nn.advertise<sensor_msgs::NavSatFix>("gps/fix", 50);
    ros::Publisher status_pub = nn.advertise<rtk_msgs::Status>("gps/status", 50);

    ros::Subscriber gps_sub = nn.subscribe("base_station/gps/raw_data", 50, baseStationCallback);

    int n;

    //********************* rtklib stuff *********************
    rtksvrinit(&server);

    if(server.state)
    {
	ROS_FATAL("RTK -- Failed to initialize rtklib server!");
	ROS_BREAK();
    }

    gtime_t time, time0 = {0};
    
    int format[] = {STRFMT_UBX, STRFMT_UBX, STRFMT_RTCM2};

    prcopt_t options = prcopt_default;
    options.mode = 2;
    options.nf = 1;
    options.navsys = SYS_GPS | SYS_SBS;
    options.modear = 3;
    options.glomodear = 0;
    options.minfix = 3;
    options.ionoopt = IONOOPT_BRDC;
    options.tropopt = TROPOPT_SAAS;
    options.rb[0] = ecef_base_station.position.x;
    options.rb[1] = ecef_base_station.position.y;
    options.rb[2] = ecef_base_station.position.z;

    strinitcom();
    server.cycle = 10;
    server.nmeacycle = 1000;
    server.nmeareq = 0;
    for(int i=0 ; i<3 ; i++) server.nmeapos[i] = 0;
    server.buffsize = BUFFSIZE;
    for(int i=0 ; i<3 ; i++) server.format[i] = format[i];
    server.navsel = 0;
    server.nsbs = 0;
    server.nsol = 0;
    server.prcout = 0;
    rtkfree(&server.rtk);
    rtkinit(&server.rtk, &options);

    for(int i=0 ; i<3 ; i++)
    {
        server.nb[i] = server.npb[i] = 0;
        if(!(server.buff[i]=(unsigned char *)malloc(BUFFSIZE)) || !(server.pbuf[i]=(unsigned char *)malloc(BUFFSIZE)))
	{
            ROS_FATAL("RTK -- Failed to initialize rtklib server - malloc error!");
            ROS_BREAK();
        }
        for(int j=0 ; j<10 ; j++) server.nmsg[i][j] = 0;
        for(int j=0 ; j<MAXOBSBUF ; j++) server.obs[i][j].n = 0;
        
        /* initialize receiver raw and rtcm control */
        init_raw(server.raw + i);
        init_rtcm(server.rtcm + i);
        
        /* set receiver option */
        strcpy(server.raw[i].opt, "");
        strcpy(server.rtcm[i].opt, "");
        
        /* connect dgps corrections */
        server.rtcm[i].dgps = server.nav.dgps;
    }
    /* output peek buffer */
    for(int i=0 ; i<2 ; i++)
    {
        if (!(server.sbuf[i]=(unsigned char *)malloc(BUFFSIZE)))
	{
            ROS_FATAL("RTK -- Failed to initialize rtklib server - malloc error!");
            ROS_BREAK();
        }
    }

    /* set solution options */
    solopt_t sol_options[2];
    sol_options[0] = solopt_default;
    sol_options[1] = solopt_default;

    for(int i=0 ; i<2 ; i++) server.solopt[i] = sol_options[i];
    
    /* set base station position */
    for(int i=0 ; i<6 ; i++) server.rtk.rb[i] = i < 3 ? options.rb[i] : 0.0;
    
    /* update navigation data */
    for(int i=0 ; i<MAXSAT*2 ; i++) server.nav.eph[i].ttr = time0;
    for(int i=0 ; i<NSATGLO*2 ; i++) server.nav.geph[i].tof = time0;
    for(int i=0 ; i<NSATSBS*2 ; i++) server.nav.seph[i].tof = time0;
    updatenav(&server.nav);
    
    /* set monitor stream */
    server.moni = NULL;

    /* open input streams */
    int stream_type[8] = {STR_SERIAL, 0, 0, 0, 0, 0, 0, 0};
    char gps_path[64];
    sprintf(gps_path, "%s:%d:8:n:1:off", port.c_str(), baudrate);
    char * paths[] = {gps_path, "localhost:27015", "", "", "", "", "", ""};
    char * cmds[] = {"", "", ""};
    
    int rw;
    for(int i=0 ; i<8 ; i++)
    {
        rw = i < 3 ? STR_MODE_R : STR_MODE_W;
	if(stream_type[i] != STR_FILE) rw |= STR_MODE_W;
        if(!stropen(server.stream+i, stream_type[i], rw, paths[i]))
	{
            ROS_ERROR("RTK -- Failed to open stream %s", paths[i]);
            for(i-- ; i>=0 ; i--) strclose(server.stream+i);
            ROS_FATAL("RTK -- Failed to initialize rtklib server - failed to open all streams!");
            ROS_BREAK();
        }
        
        /* set initial time for rtcm and raw */
        if(i<3)
	{
            time = utc2gpst(timeget());
            server.raw[i].time = stream_type[i] == STR_FILE ? strgettime(server.stream+i) : time;
            server.rtcm[i].time = stream_type[i] == STR_FILE ? strgettime(server.stream+i) : time;
        }
    }
    
    /* sync input streams */
    strsync(server.stream, server.stream+1);
    strsync(server.stream, server.stream+2);
    
    /* write start commands to input streams */
    for(int i=0 ; i<3 ; i++)
    {
        if(cmds[i]) strsendcmd(server.stream+i, cmds[i]);
    }
    
    /* write solution header to solution streams */
    for(int i=3 ; i<5 ; i++)
    {
	unsigned char buff[1024];
    	int n;
    
    	n = outsolheads(buff, server.solopt+i-3);
    	strwrite(server.stream+i, buff, n);
    }
    //********************************************************

    obs_t obs;
    obsd_t data[MAXOBS*2];
    server.state=1;
    obs.data=data;
    double tt;
    unsigned int tick;
    int fobs[3] = {0};

    server.tick = tickget();

    ROS_DEBUG("RTK -- Initialization complete.");

    ros::Rate r(rate);
    while(ros::ok())
    {
        tick = tickget();

        unsigned char *p = server.buff[RTK_ROBOT]+server.nb[RTK_ROBOT];
        unsigned char *q = server.buff[RTK_ROBOT]+server.buffsize;
        
        ROS_DEBUG("RTK -- Getting data from GPS...");
        /* read receiver raw/rtcm data from input stream */
        n = strread(server.stream, p, q-p);

        /* write receiver raw/rtcm data to log stream */
        strwrite(server.stream+5, p, n);
        server.nb[RTK_ROBOT] += n;

        /* save peek buffer */
        rtksvrlock(&server);
        n = n < server.buffsize - server.npb[RTK_ROBOT] ? n : server.buffsize - server.npb[RTK_ROBOT];
        memcpy(server.pbuf[RTK_ROBOT] + server.npb[RTK_ROBOT], p, n);
        server.npb[RTK_ROBOT] += n;
        rtksvrunlock(&server);

        ROS_DEBUG("RTK -- Decoding GPS data...");
        /* decode data */
        fobs[RTK_ROBOT] = decoderaw(&server, RTK_ROBOT);
        fobs[RTK_BASE_STATION] = decoderaw(&server, RTK_BASE_STATION);

        ROS_DEBUG("RTK -- Got %d observations.", fobs[RTK_ROBOT]);
        /* for each rover observation data */
        for(int i=0 ; i<fobs[RTK_ROBOT] ; i++)
        {
            obs.n = 0;
            for(int j=0 ; j<server.obs[RTK_ROBOT][i].n && obs.n<MAXOBS*2 ; j++)
            {
                obs.data[obs.n++] = server.obs[RTK_ROBOT][i].data[j];
            }
            for(int j=0 ; j<server.obs[1][0].n && obs.n<MAXOBS*2 ; j++)
            {
                obs.data[obs.n++] = server.obs[1][0].data[j];
            }
	    
            ROS_DEBUG("RTK -- Calculating RTK positioning...");
            /* rtk positioning */
            rtksvrlock(&server);
            rtkpos(&server.rtk, obs.data, obs.n, &server.nav);
            rtksvrunlock(&server);

            sensor_msgs::NavSatFix gps_msg;
            gps_msg.header.stamp = ros::Time::now();
            gps_msg.header.frame_id = gps_frame_id;

            rtk_msgs::Status status_msg;
            status_msg.stamp = gps_msg.header.stamp;

            if(server.rtk.sol.stat != SOLQ_NONE)
            {
                /* adjust current time */
                tt = (int)(tickget()-tick)/1000.0+DTTOL;
                timeset(gpst2utc(timeadd(server.rtk.sol.time,tt)));
                
                /* write solution */
                unsigned char buff[1024];
                n = outsols(buff, &server.rtk.sol, server.rtk.rb, server.solopt);
        	
                if(n==141 && buff[0]>'0' && buff[0]<'9')
                {
                    int ano,mes,dia,horas,minutos,Q,nsat;
                    double segundos,lat,longi,alt,sde,sdn,sdu,sdne,sdeu,sdun;

                    sscanf((const char *)(buff),"%d/%d/%d %d:%d:%lf %lf %lf %lf %d %d %lf %lf %lf %lf %lf %lf", &ano, &mes, &dia, &horas, &minutos, &segundos, &lat, &longi, &alt, &Q, &nsat, &sdn, &sde, &sdu, &sdne, &sdeu, &sdun);

                    gps_msg.latitude = lat;
                    gps_msg.longitude = longi;
                    gps_msg.altitude = alt;

                    gps_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;
                    gps_msg.position_covariance[0] = sde + ecef_base_station.position_covariance[0];
                    gps_msg.position_covariance[1] = sdne + ecef_base_station.position_covariance[1];
                    gps_msg.position_covariance[2] = sdeu + ecef_base_station.position_covariance[2];
                    gps_msg.position_covariance[3] = sdne + ecef_base_station.position_covariance[3];
                    gps_msg.position_covariance[4] = sdn + ecef_base_station.position_covariance[4];
                    gps_msg.position_covariance[5] = sdun + ecef_base_station.position_covariance[5];
                    gps_msg.position_covariance[6] = sdeu + ecef_base_station.position_covariance[6];
                    gps_msg.position_covariance[7] = sdun + ecef_base_station.position_covariance[7];
                    gps_msg.position_covariance[8] = sdu + ecef_base_station.position_covariance[8];

                    gps_msg.status.status = Q==5 ? sensor_msgs::NavSatStatus::STATUS_FIX : sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
                    gps_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

		    status_msg.fix_quality = Q;
		    status_msg.number_of_satellites = nsat;
                }
            }
            else
            {
                gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
                gps_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

            }

            ROS_DEBUG("RTK -- Publishing ROS msg...");
            gps_pub.publish(gps_msg);
	    status_pub.publish(status_msg);
        }

        ros::spinOnce();
        r.sleep();
    }

    return(0);
}

// EOF
