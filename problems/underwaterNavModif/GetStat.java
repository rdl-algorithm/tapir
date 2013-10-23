import java.io.*;
import java.util.*;

public class GetStat {
	public static void main(String args[]) {
		if (args.length < 1) {
			System.out.println("Usage : java GetStat inputFileName");
			System.exit(1);
		}
		
		try {		
			BufferedReader in = new BufferedReader(new FileReader(args[0]));
			String inStr, tmpStr;
			StringTokenizer tok;
			double tmp;
			double[] statSum = new double[5];
			double[] statSqSum = new double[5];
			for (int i = 0; i < 5; i++) {
				statSum[i] = 0.0;
				statSqSum[i] = 0.0;
			}
			int nData = 0;			
			
			while ((inStr = in.readLine()) != null) {
				tok = new StringTokenizer(inStr);
				for (int i = 0; i < 5; i++) {
					tmpStr = tok.nextToken();
					tmp = Double.parseDouble(tmpStr);
					statSum[i] = statSum[i] + tmp;
					statSqSum[i] = statSqSum[i] + tmp*tmp;
				}
				nData ++;
			}
			System.out.print(nData + "\t");
			for (int i = 0; i < 5; i++) {
				double avg = statSum[i] / (double)nData;
				double sqAvg = statSqSum[i] / (double)nData;
				System.out.print(statSum[i] + "\t" + avg + "\t" + 
						(sqAvg - avg*avg) + "\t");
			}			
			System.out.println("");
		}
		catch (IOException e) {
			System.err.println("Error: " + e.toString() + "\n");
		}
	}
}