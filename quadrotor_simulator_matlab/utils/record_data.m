function record_data(state,time,qd_id)

dataset = [time,state];
mkdir('../dataset')
path = append("../dataset/quad",num2str(qd_id),'.mat')
save(path,'dataset','qd_id');

end