package minicpbp.examples.ClassicalAIPlanning;

import minicpbp.engine.core.IntVar;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;

public class SolutionWriter {

    private boolean printToSout;
    private FileWriter csvWriter;
    private int solutionCounter = 0;

    public SolutionWriter(boolean printToSout, String csvFileName) {
        this.printToSout = printToSout;
        if(csvFileName == null){
            csvWriter = null;
            return;
        }
        try {
            File file = new File(csvFileName);
            file.delete();
            file.createNewFile();
            csvWriter = new FileWriter(file);
            csvWriter.write("sol#,cost,plan\n");
        } catch (IOException ioException){
            System.out.println("An error occurred when creating the solution output file");
            ioException.printStackTrace();
        }
    }

    public void newSolution(IntVar[] plan, int planCost) {
        solutionCounter++;
        if (printToSout) {
            System.out.println("--- Found plan of length " + plan.length + " and of cost " + planCost);
            System.out.println("Solution #" + solutionCounter);
            System.out.println("Plan: " + Arrays.toString(plan));
            System.out.flush();
        }
        if (csvWriter != null) {
            try {
                csvWriter.write(solutionCounter+","+planCost+",");
                csvWriter.write("\"[");
                csvWriter.write(Integer.toString(plan[0].min()));
                for (int i = 1; i < plan.length; i++) {
                    csvWriter.write(",");
                    csvWriter.write(Integer.toString(plan[i].min()));
                }
                csvWriter.write("]\"\n");
                csvWriter.flush();

            } catch (IOException ioException){
                System.out.println("An error occurred when writing the solution to the output file");
                ioException.printStackTrace();
            }
        }
    }

    public void close(){
        try {
            csvWriter.close();
        } catch (IOException ioException){
            System.out.println("An error occurred when closing the solution output file");
            ioException.printStackTrace();
        }
    }
}
