package minicpbp.examples.ClassicalAIPlanning.Experiment;

import minicpbp.engine.core.IntVar;
import minicpbp.engine.core.Solver;
import minicpbp.examples.ClassicalAIPlanning.PlanningInstance;
import minicpbp.examples.ClassicalAIPlanning.PlanningModel;

import java.io.IOException;
import java.io.PrintStream;
import java.util.HashMap;
import java.util.Objects;


public class InitialisationMarginals {

    public static void main(String[] args) throws IOException {

        HashMap<String,String> opt = new HashMap<>();

        opt.put("output",null);

        for (int i = 0; i < args.length; i++) {
            switch (args[i]){
                case "-i":
                    opt.put("instance",args[++i]);
                    break;
                case "-o":
                    opt.put("output",args[++i]);
                    break;
                case "-bp":
                    opt.put("bp","bp");
                    break;
                case "-bp-iter":
                    opt.put("bp",args[++i]);
                    break;
            }
        }

        PlanningInstance instance = new PlanningInstance(opt.get("instance"));
        PlanningModel model = new PlanningModel(instance, instance.minPlanLength, instance.getUpperBoundCost());
        Solver cp = model.getSolver();


        PrintStream stream = System.out;
        cp.setTraceBPFlag(true);

        if (opt.get("output") != null) {
            stream = new PrintStream(opt.get("output"));
            cp.setTraceBPFlag(true, stream);
        }
        for (IntVar intVar : model.getAction()) {
            intVar.setForBranching(true);
        }
        cp.fixPoint();
        cp.setMode(Solver.PropaMode.SBP);

        if(Objects.equals(opt.get("bp"), "bp")){
            System.out.println("bp");
            cp.beliefPropa();
        } else{
            System.out.println("vanillaBP");
            cp.vanillaBP(Integer.parseInt(opt.get("bp")));
        }
        stream.close();
    }
}
