#include "piper_workbench/workbench_state.hpp"
#include <cassert>
int main(){
 QVector<double> target{.1,1.,-1.2,0.,0.,0.};bool planCurrent=true;
 for(int i=0;i<200;++i)if(WorkbenchState::targetChanged(target,target))planCurrent=false;
 assert(planCurrent);auto moved=target;moved[3]+=.01;assert(WorkbenchState::targetChanged(target,moved));
 QJsonObject choices{{"intrinsics:d455",QJsonObject{{"backup","intrinsics-old"}}},{"tcp:d455",QJsonObject{{"backup","tcp-old"}}}};
 assert(WorkbenchState::selection(choices,"intrinsics","d455")["backup"]=="intrinsics-old");
 assert(WorkbenchState::selection(choices,"tcp","dabai")["backup"]=="tcp-old");
 assert(WorkbenchState::logSignature("arm","[1792395876.22] error 42")==
        WorkbenchState::logSignature("arm","[1792395889.77] error 99"));
 assert(WorkbenchState::logSignature("arm","same")!=WorkbenchState::logSignature("detector:dual","same"));
 assert(WorkbenchState::logSignature("workbench","Load package://rviz_common/icons/ok.png")==
        WorkbenchState::logSignature("workbench","Load package://rviz_common/icons/warning.png"));
}
