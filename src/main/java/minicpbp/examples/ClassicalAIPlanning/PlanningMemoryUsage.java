package minicpbp.examples.ClassicalAIPlanning;

import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;

public class PlanningMemoryUsage {

    private MemoryMXBean memoryBean;

    private final double MEAGABYTE = (1024*1024);
    private final String unit = " MB";


    public PlanningMemoryUsage() {
        memoryBean = ManagementFactory.getMemoryMXBean();
    }

    private double getHeapUsage(){
        return memoryBean.getHeapMemoryUsage().getUsed()/ MEAGABYTE;
    }

    private double getNonHeapUsage(){
        return memoryBean.getNonHeapMemoryUsage().getUsed()/ MEAGABYTE;
    }

    public void printUsage(){
        printUsage("");
    }

    public void printUsage(String step){
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
        System.out.printf("[non-heap:%.3f%s | heap:%.3f%s]%s\n",nonHeapMemory,unit,heapMemory,unit,s);
    }

}
