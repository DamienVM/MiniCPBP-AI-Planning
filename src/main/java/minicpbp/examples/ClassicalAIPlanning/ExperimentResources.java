package minicpbp.examples.ClassicalAIPlanning;

import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.lang.management.ThreadMXBean;

public class ExperimentResources {

    private MemoryMXBean memoryBean;
    private ThreadMXBean threadBean;

    private final double MEAGABYTE = (1024*1024);
    private final String unit = "MB";
    public final long startCpuTime;
    public final long startWallTime;


    public ExperimentResources() {
        memoryBean = ManagementFactory.getMemoryMXBean();
        threadBean = ManagementFactory.getThreadMXBean();
        startCpuTime = threadBean.getThreadCpuTime(Thread.currentThread().getId());
        startWallTime = System.currentTimeMillis();
    }

    private double getHeapUsage(){
        return memoryBean.getHeapMemoryUsage().getUsed()/ MEAGABYTE;
    }

    private double getNonHeapUsage(){
        return memoryBean.getNonHeapMemoryUsage().getUsed()/ MEAGABYTE;
    }

    public void printStats(String step){
        printMemory(getNonHeapUsage(),getHeapUsage(), step);
    }

    public void printMaxHeapMemory(){
        System.out.println("Maximum heap memory : " + (memoryBean.getHeapMemoryUsage().getMax() / MEAGABYTE) + unit);
    }

    private void printMemory(double nonHeapMemory, double heapMemory, String step){
        String s = "" ;
        if (!step.isEmpty()){
            s = " ("+step+")";
        }
        System.out.printf("[t:%s, non-heap:%.3f%s, heap:%.3f%s]%s\n",elapsedCPUTimeStr(),nonHeapMemory,unit,heapMemory,unit,s);
    }

    /**
     * Returns the elapsed time in seconds.
     */
    public double elapsedCPUTime(){
        return (threadBean.getThreadCpuTime(Thread.currentThread().getId()) - startCpuTime) * 1e-9;
    }

    public String elapsedCPUTimeStr(){
        return String.format( "%.3fs", elapsedCPUTime() );
    }


    public String elapsedWallTime(){
        return (System.currentTimeMillis()-startWallTime)*1e-3+" s";
    }

}
