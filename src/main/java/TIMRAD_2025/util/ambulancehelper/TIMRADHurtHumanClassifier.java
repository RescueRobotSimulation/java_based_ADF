package TIMRAD_2025.util.ambulancehelper;

import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.WorldInfo;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;

import java.util.*;

public class TIMRADHurtHumanClassifier {
    
        private Set<StandardEntity> myGoodHumans = new HashSet<>();// should think about these.
        private ArrayList<StandardEntity> healthyHumans = new ArrayList<>();
        private ArrayList<Civilian> shouldCarryToRefugeCivilians = new ArrayList<>();
        private ArrayList<StandardEntity> selectedHumans = new ArrayList<>();
        private ArrayList<StandardEntity> myBadHumans = new ArrayList<>();// shouldn't think about these.

        private ArrayList<StandardEntity> unReachableHumans = new ArrayList<>(); //todo should Consider Humans
        private Map<EntityID, Integer> unreachableHumanTime = new HashMap<>();

        private Random rnd = new Random(345);
        private WorldInfo worldInfo;
        private AgentInfo agentInfo;

    public TIMRADHurtHumanClassifier(WorldInfo wi, AgentInfo agentInfo) {
        this.worldInfo = wi;
        this.agentInfo = agentInfo;
    }

        public void updateGoodHumanList(Collection<StandardEntity> civilians) {

        myGoodHumans.clear();
        myGoodHumans.addAll(civilians);
        Human human;

        myGoodHumans.removeAll(selectedHumans);
        myGoodHumans.removeAll(myBadHumans);
        myGoodHumans.removeAll(healthyHumans);
        myGoodHumans.removeAll(unReachableHumans);

        ArrayList<Human> lowInfoHumans = new ArrayList<Human>();
        for (StandardEntity standardEntity : myGoodHumans) {
            Human h = (Human) standardEntity;
            if (h.isPositionDefined() && h.isHPDefined() && h.isDamageDefined() && h.isBuriednessDefined()) {
                StandardEntity posEntity = worldInfo.getEntity(h.getPosition());
                if ((posEntity instanceof Building) && ((Building) posEntity).isBrokennessDefined() && h.isBuriednessDefined() && h.getBuriedness() >0) {
                    lowInfoHumans.add(h);
                    //视线范围内没有这个人
                } else if (agentInfo.getPosition().equals(h.getPosition()) && !worldInfo.getChanged().getChangedEntities().contains(h.getID())) {
                    lowInfoHumans.add(h);
                    myBadHumans.add(h);
                } else if (h.getHP() == 0 || (h instanceof Civilian && h.getPosition(worldInfo.getRawWorld()) instanceof Refuge) || !(h.getPosition(worldInfo.getRawWorld()) instanceof Area)) {
                    lowInfoHumans.add(h);
                } else if (h.getDamage() > 0 && h.getBuriedness() == 0) {
                    {
                        if (!(h instanceof Civilian) || (worldInfo.getEntitiesOfType(StandardEntityURN.REFUGE).isEmpty() && !(h.getPosition(worldInfo.getRawWorld()) instanceof Building))) {
                            lowInfoHumans.add(h);
                        }
                    }
                } else if ((h.getDamage() == 0 && h.getBuriedness() == 0)) {
                    if (h instanceof Civilian && worldInfo.getRawWorld().getEntity(h.getPosition()) instanceof Refuge) {
                        lowInfoHumans.add(h);
                    } else if ((h instanceof Civilian) && h.getPosition(worldInfo.getRawWorld()) instanceof Road) {
                        lowInfoHumans.add(h);
                    } else if (!(h instanceof Civilian)) {
                        lowInfoHumans.add(h);
                    } else if ((!(h.getPosition(worldInfo.getRawWorld()) instanceof Building) && h.getHP() == 10000)) {
                        lowInfoHumans.add(h);
                    }
                } else if (!(h.getPosition(worldInfo.getRawWorld()) instanceof Area)) {
                    lowInfoHumans.add(h);
                }
            } else {
                lowInfoHumans.add(h);
            }
        }

        myGoodHumans.removeAll(lowInfoHumans);
    }

        public Set<StandardEntity> getMyGoodHumans() {
        return myGoodHumans;
    }
}
