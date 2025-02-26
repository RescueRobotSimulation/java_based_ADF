package TIMRAD_2025.util.ambulancehelper;

import adf.core.agent.info.WorldInfo;
import rescuecore2.standard.entities.StandardEntity;

import java.util.Comparator;

public class TIMRADDistanceSorter implements Comparator<StandardEntity> {
    private StandardEntity reference;
    private WorldInfo worldInfo;

    public TIMRADDistanceSorter(WorldInfo wi, StandardEntity reference) {
        this.reference = reference;
        this.worldInfo = wi;
    }

    public int compare(StandardEntity a, StandardEntity b) {
        int d1 = this.worldInfo.getDistance(this.reference, a);
        int d2 = this.worldInfo.getDistance(this.reference, b);
        return d1 - d2;
    }
}
