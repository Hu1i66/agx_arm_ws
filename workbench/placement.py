import copy,math,time
from .settings import JsonStore

DEFAULT={'schema_version':1,'reference_poses':{'standby_joints_rad':[0.]*6,
'dabai_observe_joints_rad':[0.,1.0669023184516138,-1.1636808254746993,0.,1.2184841639873214,0.]},
'plans':[{'id':'legacy-default','name':'原放置点','via_joints_rad':[],
'release_joints_rad':[1.9992746581595044,1.5727510955571302,-1.0801144608892108,-0.008045967851693859,1.0517179539592632,0.022270401255447644]}],
'selection':{'mode':'single','default_plan_id':'legacy-default','class_to_plan_id':{}}}

def joints(values):
    if len(values)!=6 or not all(isinstance(x,(int,float)) and math.isfinite(x) for x in values):
        raise ValueError('关节位置必须为六个有限弧度值')
    return list(values)

class PlacementStore(JsonStore):
    def __init__(self,path):super().__init__(path,DEFAULT)
    def validate(self,data):
        super().validate(data)
        ids=set()
        for plan in data['plans']:
            if not plan['id'] or plan['id'] in ids:raise ValueError('方案 ID 为空或重复')
            ids.add(plan['id']);joints(plan['release_joints_rad'])
            if len(plan['via_joints_rad'])>5:raise ValueError('途经点最多五个')
            for row in plan['via_joints_rad']:joints(row)
        selection=data['selection']
        if selection['default_plan_id'] not in ids:raise ValueError('必须指定现存的默认放置方案')
        selection['class_to_plan_id']={k:v if v in ids else selection['default_plan_id'] for k,v in selection['class_to_plan_id'].items()}
        for pose in data['reference_poses'].values():joints(pose)

    def save(self,data):
        data=copy.deepcopy(data)
        self.validate(data)
        return super().save(data)

def resolve_placement(class_name,camera_id,settings):
    data=copy.deepcopy(settings);sel=data['selection'];pid=sel['default_plan_id']
    if sel['mode']=='class':pid=sel['class_to_plan_id'].get(class_name,pid)
    plan=next(p for p in data['plans'] if p['id']==pid)
    plan['normal_return_joints_rad']=data['reference_poses']['dabai_observe_joints_rad' if camera_id=='dabai' else 'standby_joints_rad']
    plan['snapshot_at']=time.time();return plan
