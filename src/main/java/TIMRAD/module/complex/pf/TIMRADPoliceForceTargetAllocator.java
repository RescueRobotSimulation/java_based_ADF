package TIMRAD.module.complex.pf;

import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.component.module.complex.PoliceTargetAllocator;
import rescuecore2.worldmodel.EntityID;

import java.util.HashMap;
import java.util.Map;

/**
 * @author Shiva
 */

public class TIMRADPoliceForceTargetAllocator extends PoliceTargetAllocator {

    public TIMRADPoliceForceTargetAllocator(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager, DevelopData developData) {
        super(ai, wi, si, moduleManager, developData);
    }

    @Override
    public PoliceTargetAllocator resume(PrecomputeData precomputeData) {
        super.resume(precomputeData);
        if (this.getCountResume() >= 2) {
            return this;
        }
        return this;
    }

    @Override
    public PoliceTargetAllocator preparate() {
        super.preparate();
        if (this.getCountPrecompute() >= 2) {
            return this;
        }
        return this;
    }
    @Override
    public PoliceTargetAllocator updateInfo(MessageManager mm) {
        super.updateInfo(mm);

        if (!this.hasRegisterRequest) {
            final var index = new StandardMessageBundle().getMessageClassList().size() + 1;
            mm.registerMessageClass(index, MessageClearRequest.class);
            this.hasRegisterRequest = true;
        }

        if (this.getCountUpdateInfo() >= 2) {
            return this;
        }
        return this;
    }

    @Override
    public Map<EntityID, EntityID> getResult() {
        // ایجاد ماتریس هزینه (مسافت)
        double[][] costMatrix = createCostMatrix();

        // اعمال الگوریتم Hungarian برای تخصیص بهینه
        int[] assignment = hungarianAlgorithm(costMatrix);
        Map<EntityID, EntityID> result = new HashMap<>();
        for (int i = 0; i < assignment.length; i++) {
            if (assignment[i] != -1) {
                result.put(policeUnits.get(i), targets.get(assignment[i]));
            }
        }
        return result;
    }

    // ایجاد ماتریس هزینه که برای تخصیص وظایف استفاده می‌شود
    private double[][] createCostMatrix() {
        int numPolice = policeUnits.size();
        int numTargets = targets.size();
        double[][] costMatrix = new double[numPolice][numTargets];

        for (int i = 0; i < numPolice; i++) {
            for (int j = 0; j < numTargets; j++) {
                // در اینجا، مسافت به عنوان هزینه در نظر گرفته می‌شود
                costMatrix[i][j] = calculateCost(policeUnits.get(i), targets.get(j));
            }
        }
        return costMatrix;
    }

    //محاسبه هزینه (مسافت)
    private double calculateCost(EntityID policeUnit, EntityID target) {

        return policeUnit.getID() - target.getID();
    }

    // الگوریتم Hungarian برای تخصیص وظایف
    private int[] hungarianAlgorithm(double[][] costMatrix) {
        int n = costMatrix.length;
        int m = costMatrix[0].length;
        double[][] u = new double[n][1];
        double[][] v = new double[m][1];
        int[] p = new int[m];
        int[] way = new int[m];
        int[] assignment = new int[n];

        for (int i = 0; i < n; i++) {
            p[0] = i;
            int j0 = 0;
            double[] minv = new double[m];
            boolean[] used = new boolean[m];
            for (int j = 0; j < m; j++) {
                minv[j] = Double.MAX_VALUE;
                used[j] = false;
            }
            do {
                int j1 = p[j0];
                double delta = Double.MAX_VALUE;
                int j2 = -1;
                used[j1] = true;
                for (int j = 1; j < m; j++) {
                    if (!used[j]) {
                        double cur = costMatrix[j0][j] - u[j0][0] - v[j];
                        if (cur < minv[j]) {
                            minv[j] = cur;
                            way[j] = j1;
                        }
                        if (minv[j] < delta) {
                            delta = minv[j];
                            j2 = j;
                        }
                    }
                }
                for (int j = 0; j < m; j++) {
                    if (used[j]) {
                        u[p[j]] += delta;
                        v[j] -= delta;
                    } else {
                        minv[j] -= delta;
                    }
                }
                j0 = j2;
            } while (p[j0] != -1);
            do {
                int j1 = way[j0];
                p[j0] = p[j1];
                j0 = j1;
            } while (j0 != 0);
        }
        for (int i = 0; i < n; i++) {
            assignment[i] = p[i];
        }
        return assignment;
    }

    @Override
    public PoliceTargetAllocator calc() {
        return this;
    }
}
