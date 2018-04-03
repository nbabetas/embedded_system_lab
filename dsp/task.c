/*  ----------------------------------- DSP/BIOS Headers            */
#include <std.h>
#include <gbl.h>
#include <log.h>
#include <swi.h>
#include <sys.h>
#include <tsk.h>
#include <pool.h>

/*  ----------------------------------- DSP/BIOS LINK Headers       */
#include <failure.h>
#include <dsplink.h>
#include <platform.h>
#include <notify.h>
#include <bcache.h>
/*  ----------------------------------- Sample Headers              */
#include <pool_notify_config.h>
#include <task.h>
#include <stdlib.h>
#include <stdint.h>

#define INT_FIXED(number) (((uint16_t)number)<<8)
#define MULTIPLICATION(A,B) (uint16_t)(((uint32_t)A*(uint32_t)B+(1<<(7)))>>8)  
#define DIVISION(A,B) (uint16_t)((((uint32_t)A<<8)+(B/2))/B)

extern Uint16 MPCSXFER_BufferSize ;
unsigned char* buf;

int rows, cols, windowSize;    /* Dimension of the gaussian kernel. */
//We use static Arrays because are faster than dynamic ones
unsigned char image[76800];// Local array on DSP memory for received picture from ARM
uint16_t kernel[20]; // Local array on DSP memory for received kernel from ARM
uint16_t tempim[76800];// Local array on DSP memory for the first half of calculations from gaussian_smooth
uint16_t smoothedim[76800];// Local array on DSP memory for the second half of calculations from gaussian_smooth


static Void Task_notify (Uint32 eventNo, Ptr arg, Ptr info) ;

Int Task_create (Task_TransferInfo ** infoPtr)
{
    Int status    = SYS_OK ;
    Task_TransferInfo * info = NULL ;

    /* Allocate Task_TransferInfo structure that will be initialized
     * and passed to other phases of the application */
    if (status == SYS_OK) 
	{
        *infoPtr = MEM_calloc (DSPLINK_SEGID,
                               sizeof (Task_TransferInfo),
                               0) ; /* No alignment restriction */
        if (*infoPtr == NULL) 
		{
            status = SYS_EALLOC ;
        }
        else 
		{
            info = *infoPtr ;
        }
    }

    /* Fill up the transfer info structure */
    if (status == SYS_OK) 
	{
        info->dataBuf       = NULL ; /* Set through notification callback. */
        info->bufferSize    = MPCSXFER_BufferSize ;
        SEM_new (&(info->notifySemObj), 0) ;
    }

    /*
     *  Register notification for the event callback to get control and data
     *  buffer pointers from the GPP-side.
     */
    if (status == SYS_OK) 
	{
        status = NOTIFY_register (ID_GPP,
                                  MPCSXFER_IPS_ID,
                                  MPCSXFER_IPS_EVENTNO,
                                  (FnNotifyCbck) Task_notify,
                                  info) ;
        if (status != SYS_OK) 
		{
            return status;
        }
    }

    /*
     *  Send notification to the GPP-side that the application has completed its
     *  setup and is ready for further execution.
     */
    if (status == SYS_OK) 
	{
        status = NOTIFY_notify (ID_GPP,
                                MPCSXFER_IPS_ID,
                                MPCSXFER_IPS_EVENTNO,
                                (Uint32) 0) ; /* No payload to be sent. */
        if (status != SYS_OK) 
		{
            return status;
        }
    }

    /*
     *  Wait for the event callback from the GPP-side to post the semaphore
     *  indicating receipt of the data buffer pointer and image width and height.
     */
    
    SEM_pend (&(info->notifySemObj), SYS_FOREVER) ;//wait for semaphore in order to get databuffer
    SEM_pend (&(info->notifySemObj), SYS_FOREVER) ;//wait for semaphore in order to get rows from pool_notify_dimension
    SEM_pend (&(info->notifySemObj), SYS_FOREVER) ;//wait for semaphore in order to get cols from pool_notify_dimension
    return status ;
}

Int Task_execute (Task_TransferInfo * info)
{
// ------------------- Receive Image and windowSize---------------------------------
    SEM_pend (&(info->notifySemObj), SYS_FOREVER);
    BCACHE_inv ((Ptr)buf, MEM_SIZE, TRUE) ;
    memcpy(image,buf,rows*cols);
    NOTIFY_notify(ID_GPP,MPCSXFER_IPS_ID,MPCSXFER_IPS_EVENTNO,(Uint32)0);
    
//-------------------- Receive Kernel from ARM------------------------------------
    SEM_pend (&(info->notifySemObj), SYS_FOREVER);
    //invalidate cache
    BCACHE_inv ((Ptr)buf, MEM_SIZE, TRUE) ;
    memcpy(kernel,buf,windowSize*sizeof(uint16_t));
       
    gaussian_smooth(); // execute gaussian smooth on DSP
//-----------------------Send smoothed image to ARM----------------------------------
    memcpy(buf,smoothedim, 76800*sizeof(uint16_t)); 
    BCACHE_wb((Ptr)buf, MEM_SIZE, TRUE);
    
    NOTIFY_notify(ID_GPP,MPCSXFER_IPS_ID,MPCSXFER_IPS_EVENTNO,(Uint32)0); // inform ARM tha DSP finished
   
    return SYS_OK;
}
//------------------------- GAUSSIAN SMOOTH WITH FIXED POINT ARITHMETICS--------------------------------
void gaussian_smooth()
{   
int r, c, rr, cc,    /* Counter variables. */
        center;            /* Half of the windowsize. */
    uint32_t dot,sum;              /* Dot product summing variable. */
                      
	
    center = windowSize / 2;
	/****************************************************************************
    * Blur in the x - direction.
    ****************************************************************************/
    
    for(r=0; r<rows; r++)
    {
        for(c=0; c<cols; c++)
        {
            dot = 0;
            sum = 0;
            for(cc=(-center); cc<=center; cc++)
            {
                if(((c+cc) >= 0) && ((c+cc) < cols))
                {
                    dot += MULTIPLICATION(INT_FIXED(image[r*cols+(c+cc)]), kernel[center+cc]);
                    sum += kernel[center+cc];
                }
            }
            tempim[r*cols+c] = DIVISION(dot,sum);
        }
    }
    /****************************************************************************
    * Blur in the y - direction.
    ****************************************************************************/
    for(c=0; c<cols; c++)
    {
        for(r=0; r<rows; r++)
        {
            sum = 0;
            dot = 0;
            for(rr=(-center); rr<=center; rr++)
            {
                if(((r+rr) >= 0) && ((r+rr) < rows))
                {
                    dot += MULTIPLICATION(tempim[(r+rr)*cols+c],kernel[center+rr]);
                    sum += kernel[center+rr];
                }
            }
            smoothedim[r*cols+c] = DIVISION(dot,sum);
			
        }
    }
}
Int Task_delete (Task_TransferInfo * info)
{
    Int    status     = SYS_OK ;
    /*
     *  Unregister notification for the event callback used to get control and
     *  data buffer pointers from the GPP-side.
     */
    status = NOTIFY_unregister (ID_GPP,
                                MPCSXFER_IPS_ID,
                                MPCSXFER_IPS_EVENTNO,
                                (FnNotifyCbck) Task_notify,
                                info) ;

    /* Free the info structure */
    MEM_free (DSPLINK_SEGID,
              info,
              sizeof (Task_TransferInfo)) ;
    info = NULL ;

    return status ;
}

static Void Task_notify (Uint32 eventNo, Ptr arg, Ptr info)
{
    static int count = 0;
    Task_TransferInfo * mpcsInfo = (Task_TransferInfo *) arg ;

    (Void) eventNo ; /* To avoid compiler warning. */

    count++;
    if (count==1) {
        buf =(unsigned char*)info ;
    }
   if (count==2) {
        rows = (int)info;           
    }
     if (count==3) {
        cols = (int)info;
    }
     if (count==4) {
        windowSize = (int)info;
    }
     if (count==5){
      // kernel
    }
    SEM_post(&(mpcsInfo->notifySemObj));
}
