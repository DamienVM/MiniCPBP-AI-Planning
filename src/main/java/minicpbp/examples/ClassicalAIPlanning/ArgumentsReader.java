package minicpbp.examples.ClassicalAIPlanning;

import java.util.HashMap;

public class ArgumentsReader {

    public static HashMap<String,String> readArguments(String[] args){

        HashMap<String, String> ret = new HashMap<>();

        // Set default values
        ret.put("output", null);

        for (int i = 0; i < args.length; i++) {
            switch (args[i]) {
                case "-i":
                    ret.put("instance", args[++i]);
                    break;
                case "-o":
                    ret.put("output", args[++i]);
                    break;
                case "-search":
                    ret.put("search", args[++i]);
                    break;
                case "-branching":
                    ret.put("branching", args[++i]);
                    break;
                case "-lp":
                    ret.put("lp", args[++i]);
                    break;
                default:
                    throw new RuntimeException(String.format("Option '%s' not recognized", args[i]));
            }

        }
        return ret;
    }
}
