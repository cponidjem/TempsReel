#include "../header/functions.h"

char mode_start;

void write_in_queue(RT_QUEUE *, MessageToMon);
void confirmArena(RT_QUEUE*, bool);
void f_detectRobotLoss(int err);

void f_server(void *arg) {
    int err;
    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    err = run_nodejs("/usr/local/bin/node", "/home/pi/Interface_Robot/server.js");

    if (err < 0) {
        printf("Failed to start nodejs: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    } else {
#ifdef _WITH_TRACE_
        printf("%s: nodejs started\n", info.name);
#endif
        open_server();
        rt_sem_broadcast(&sem_serverOk);
    }
}

void f_sendToMon(void * arg) {
    int err;
    MessageToMon msg;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

#ifdef _WITH_TRACE_
    printf("%s : waiting for sem_serverOk\n", info.name);
#endif
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    while (1) {

#ifdef _WITH_TRACE_
        printf("%s : waiting for a message in queue\n", info.name);
#endif
        if (rt_queue_read(&q_messageToMon, &msg, sizeof (MessageToMon), TM_INFINITE) >= 0) {
#ifdef _WITH_TRACE_
            printf("%s : message {%s,%s} in queue\n", info.name, msg.header, msg.data);
#endif

            err = send_message_to_monitor(msg.header, msg.data);
            if(err < 0) {
                rt_mutex_acquire(&mutex_nodeJSLoss,TM_INFINITE);
                if(!nodeJSLoss) {
                    nodeJSLoss = true;
                    rt_sem_v(&sem_nodeJSLoss);
                }               
                rt_mutex_release(&mutex_nodeJSLoss);
                
            }
            free_msgToMon_data(&msg);
            rt_queue_free(&q_messageToMon, &msg);
        } else {
            printf("Error msg queue write: %s\n", strerror(-err));
        }
    }
}

void f_receiveFromMon(void *arg) {
    MessageFromMon msg;
    int err;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

#ifdef _WITH_TRACE_
    printf("%s : waiting for sem_serverOk\n", info.name);
#endif
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    do {
#ifdef _WITH_TRACE_
        printf("%s : waiting for a message from monitor\n", info.name);
#endif
        err = receive_message_from_monitor(msg.header, msg.data);
#ifdef _WITH_TRACE_
        printf("%s: msg {header:%s,data=%s} received from UI\n", info.name, msg.header, msg.data);
#endif
        if (err <= 0) {
            rt_mutex_acquire(&mutex_nodeJSLoss,TM_INFINITE);
            nodeJSLoss = true;
            rt_mutex_release(&mutex_nodeJSLoss);
            rt_sem_v(&sem_nodeJSLoss);
        } else {
        
            if (strcmp(msg.header, HEADER_MTS_COM_DMB) == 0) {
                if (msg.data[0] == OPEN_COM_DMB) { // Open communication supervisor-robot
    #ifdef _WITH_TRACE_
                    printf("%s: message open Xbee communication\n", info.name);
    #endif
                    rt_sem_v(&sem_openComRobot);
                }
            } else if (strcmp(msg.header, HEADER_MTS_DMB_ORDER) == 0) {
                if (msg.data[0] == DMB_START_WITHOUT_WD) { // Start robot
    #ifdef _WITH_TRACE_
                    printf("%s: message start robot\n", info.name);
    #endif 
                    rt_sem_v(&sem_startRobot);

                } else if ((msg.data[0] == DMB_GO_BACK)
                        || (msg.data[0] == DMB_GO_FORWARD)
                        || (msg.data[0] == DMB_GO_LEFT)
                        || (msg.data[0] == DMB_GO_RIGHT)
                        || (msg.data[0] == DMB_STOP_MOVE)) {

                    rt_mutex_acquire(&mutex_move, TM_INFINITE);
                    move = msg.data[0];
                    rt_mutex_release(&mutex_move);
    #ifdef _WITH_TRACE_
                    printf("%s: message update movement with %c\n", info.name, move);
    #endif

                }
            } else if (strcmp(msg.header, HEADER_MTS_CAMERA) == 0){
                if(msg.data[0] == CAM_OPEN) {
                    rt_sem_v(&sem_openCamera);
                } else if (msg.data[0] == CAM_CLOSE) {
                    rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                    close_camera(&camera);
                    rt_mutex_release(&mutex_camera);
                    MessageToMon msg;
                    set_msgToMon_header(&msg, HEADER_STM_ACK);
                    write_in_queue(&q_messageToMon, msg);
                    
                } else if (msg.data[0] == CAM_ASK_ARENA) {
                    rt_sem_v(&sem_findArena);
                } else if (msg.data[0] == CAM_ARENA_CONFIRM) {
                    confirmArena(&q_confirmArena, true); 
                } else if (msg.data[0] == CAM_ARENA_INFIRM) {
                    confirmArena(&q_confirmArena, false); 
                } else if (msg.data[0] == CAM_COMPUTE_POSITION) {
                    rt_mutex_acquire(&mutex_continueDetectPos, TM_INFINITE);
                    continueDetectPos = true;
                    rt_mutex_release(&mutex_continueDetectPos);
                } else if (msg.data[0] == CAM_STOP_COMPUTE_POSITION) {
                    rt_mutex_acquire(&mutex_continueDetectPos, TM_INFINITE);
                    continueDetectPos = false;
                    rt_mutex_release(&mutex_continueDetectPos);
                }

            }
        }
    } while (err > 0);

}

void f_openComRobot(void * arg) {
    int err;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while (1) {
#ifdef _WITH_TRACE_
        printf("%s : Wait sem_openComRobot\n", info.name);
#endif
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
#ifdef _WITH_TRACE_
        printf("%s : sem_openComRobot arrived => open communication robot\n", info.name);
#endif
        err = open_communication_robot();
        if (err == 0) {
#ifdef _WITH_TRACE_
            printf("%s : the communication is opened\n", info.name);
#endif
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_ACK);
            write_in_queue(&q_messageToMon, msg);
        } else {
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
            write_in_queue(&q_messageToMon, msg);
        }
    }
}

void f_startRobot(void * arg) {
    int err;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while (1) {
#ifdef _WITH_TRACE_
        printf("%s : Wait sem_startRobot\n", info.name);
#endif
        rt_sem_p(&sem_startRobot, TM_INFINITE);
#ifdef _WITH_TRACE_
        printf("%s : sem_startRobot arrived => Start robot\n", info.name);
#endif
        err = send_command_to_robot(DMB_START_WITHOUT_WD);
        f_detectRobotLoss(err);
        if (err == 0) {
#ifdef _WITH_TRACE_
            printf("%s : the robot is started\n", info.name);
#endif
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
           
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_ACK);
            write_in_queue(&q_messageToMon, msg);
        } else {
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
            write_in_queue(&q_messageToMon, msg);
        }
    }
}

void f_move(void *arg) {
    /* INIT */
    int err;
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /* PERIODIC START */
#ifdef _WITH_TRACE_
    printf("%s: start period\n", info.name);
#endif
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    while (1) {
#ifdef _WITH_TRACE_
        printf("%s: Wait period \n", info.name);
#endif
        rt_task_wait_period(NULL);
#ifdef _WITH_TRACE_
        printf("%s: Periodic activation\n", info.name);
        printf("%s: move equals %c\n", info.name, move);
#endif
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        if (robotStarted) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            err = send_command_to_robot(move);
            rt_mutex_release(&mutex_move);
            f_detectRobotLoss(err);
#ifdef _WITH_TRACE_
            printf("%s: the movement %c was sent\n", info.name, move);
#endif            
        }
        rt_mutex_release(&mutex_robotStarted);
    }
}

void f_detectNodeJSLoss(void *arg){
     /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    while(1){
        rt_sem_p(&sem_nodeJSLoss, TM_INFINITE);
        rt_mutex_acquire(&mutex_nodeJSLoss,TM_INFINITE);
        if (nodeJSLoss) {
            printf("La communication avec NodeJS a été perdue \n");
        }
        rt_mutex_release(&mutex_nodeJSLoss);
    }
}

void f_openCamera(void *arg) {
    int err;
    
     /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    while(1){
        rt_sem_p(&sem_openCamera, TM_INFINITE);
        rt_mutex_acquire(&mutex_camera,TM_INFINITE);
        err = open_camera(&camera);
        rt_mutex_release(&mutex_camera);
        if (err == 0){
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_ACK);
            write_in_queue(&q_messageToMon, msg);           
        } else {
             MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
            write_in_queue(&q_messageToMon, msg);
        }
        
        
    }
}

void f_sendImage(void *arg){
    Arene arena;
    Position tabPosition[10];
     /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
     /* PERIODIC START */
#ifdef _WITH_TRACE_
    printf("%s: start period\n", info.name);
#endif
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    while (1) {
#ifdef _WITH_TRACE_
        printf("%s: Wait period \n", info.name);
#endif
        rt_task_wait_period(NULL);
#ifdef _WITH_TRACE_
        printf("%s: Periodic activation\n", info.name);
#endif  
        rt_mutex_acquire(&mutex_periodicImage,TM_INFINITE);
        rt_mutex_acquire(&mutex_camera,TM_INFINITE);
        if(camera.isOpened() && periodicImage){
            Image image;
            Jpg jpgImage;
            get_image(&camera,&image);
            rt_mutex_acquire(&mutex_savedArena,TM_INFINITE);
            arena = savedArena;
            rt_mutex_release(&mutex_savedArena);
            rt_mutex_acquire(&mutex_continueDetectPos, TM_INFINITE);
            if(continueDetectPos){
                if(arena.area()>0){
                    MessageToMon msg;
                    set_msgToMon_header(&msg,HEADER_STM_POS);
                    if(detect_position(&image,tabPosition,&arena)){
                        draw_position(&image,&image,&tabPosition[0]);
                        set_msgToMon_data(&msg,&tabPosition[0]);
                    } else {
                        
                        Position position;
                        position.center.x = -1;
                        position.center.y = -1;
                        position.angle = -1;
                        set_msgToMon_data(&msg,&position);                        
                    }
                    write_in_queue(&q_messageToMon,msg);
                }  
                
            }
            rt_mutex_release(&mutex_continueDetectPos);
            if(arena.area()>0){
                 draw_arena(&image,&image,&arena);
            }   
            
            compress_image(&image,&jpgImage);
            /*writing IMAGE in sendToMon queue doesn't work 
            you have to send it directly with the monitor function
            MessageToMon msg;
            set_msgToMon_header(&msg,HEADER_STM_IMAGE);
            set_msgToMon_data(&msg,&jpgImage);
            write_in_queue(&q_messageToMon,msg);*/
            send_message_to_monitor(HEADER_STM_IMAGE, &jpgImage);
#ifdef _WITH_TRACE_
    printf("%s: the image was sent\n", info.name);
#endif            
        }
        rt_mutex_release(&mutex_camera);
        rt_mutex_release(&mutex_periodicImage);
    }
}

void f_findArena(void *arg){
    int err;
    bool arenaOk;
    Image image;
    Jpg jpgImage;
    Arene arena;
    
     /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    while(1){
        rt_sem_p(&sem_findArena, TM_INFINITE);
        rt_mutex_acquire(&mutex_periodicImage,TM_INFINITE);
        periodicImage = false;
        rt_mutex_release(&mutex_periodicImage);
        rt_mutex_acquire(&mutex_camera,TM_INFINITE);
        if(camera.isOpened()){
            get_image(&camera,&image);
        }        
        rt_mutex_release(&mutex_camera);
        err= detect_arena(&image,&arena);
        if(err == 0){
            draw_arena(&image,&image,&arena);
            compress_image(&image,&jpgImage);
            send_message_to_monitor(HEADER_STM_IMAGE,&jpgImage);
            if (rt_queue_read(&q_confirmArena, &arenaOk, sizeof (bool), TM_INFINITE) >= 0) {
                if(arenaOk){
                    rt_mutex_acquire(&mutex_savedArena,TM_INFINITE);
                    savedArena = arena;
                    rt_mutex_release(&mutex_savedArena);
                }
            } else {
                printf("Error msg queue write: %s\n", strerror(-err));
            }
        } else {
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
            write_in_queue(&q_messageToMon, msg);
        }      
        rt_mutex_acquire(&mutex_periodicImage,TM_INFINITE);
        periodicImage = true;
        rt_mutex_release(&mutex_periodicImage);
    }
}

void f_checkBattery(void *arg) {
    /* INIT */
    int err1;
    int err;
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /* PERIODIC START */
#ifdef _WITH_TRACE_
    printf("%s: start period\n", info.name);
#endif
    rt_task_set_periodic(NULL, TM_NOW, 500000000);
    while (1) {
#ifdef _WITH_TRACE_
        printf("%s: Wait period \n", info.name);
#endif
        rt_task_wait_period(NULL);
#ifdef _WITH_TRACE_
        printf("%s: Periodic activation\n", info.name);
#endif
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        if (robotStarted) {
            err1 = send_command_to_robot(DMB_GET_VBAT);
            err = err1 + 48;
            f_detectRobotLoss(err1);
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_BAT);
            set_msgToMon_data(&msg, &err);
            write_in_queue(&q_messageToMon, msg);
#ifdef _WITH_TRACE_
    printf("%s: battery level %c was sent\n", info.name, &err);
#endif 
        }
        rt_mutex_release(&mutex_robotStarted);       
    }
        
}

void confirmArena(RT_QUEUE *queue, bool arenaOk){
    void *buff;
    buff = rt_queue_alloc(&q_messageToMon, sizeof (bool));
    memcpy(buff, &arenaOk, sizeof (bool));
    rt_queue_send(queue, buff, sizeof (bool), Q_NORMAL);
}

void f_detectRobotLoss(int err){     
           
    if(err>=0){
         rt_mutex_acquire(&mutex_compteur, TM_INFINITE);
        cpt=0;
         rt_mutex_release(&mutex_compteur);
#ifdef _WITH_TRACE_
    printf("compteur = %d \n", cpt);
#endif          
    }else{ 
        
        
#ifdef _WITH_TRACE_
    printf("compteur = %d \n", cpt);
#endif 
        rt_mutex_acquire(&mutex_compteur, TM_INFINITE);
        if(cpt==3){
           MessageToMon msg;
           set_msgToMon_header(&msg, HEADER_STM_LOST_DMB);
           write_in_queue(&q_messageToMon, msg);
           close_communication_robot();
           robotStarted=0;
        }else{
            cpt++;
        }
        rt_mutex_release(&mutex_compteur);
    }       
}
void write_in_queue(RT_QUEUE *queue, MessageToMon msg) {
    void *buff;
    buff = rt_queue_alloc(queue, sizeof (MessageToMon));
    memcpy(buff, &msg, sizeof (MessageToMon));
    rt_queue_send(queue, buff, sizeof (MessageToMon), Q_NORMAL);
}