var btn_SaveButton=document.querySelector("#SaveButton");
var btn_SaveStandButton=document.querySelector("#SaveStandButton");
var infoModal=document.querySelector("#infoModal");
var infoModal2=document.querySelector("#infoModal2");
var close_savedata=document.querySelector("#savedata");
var close_savestanddata=document.querySelector("#savestanddata");
var close_no1=document.querySelector("#nosavedata1");
var close_no2=document.querySelector("#nosavedata2");
btn_SaveStandButton.addEventListener("click", function(){
  infoModal.showModal();
  console.log('test ');
},false);
btn_SaveButton.addEventListener("click", function(){
    infoModal2.showModal();
    console.log('test ');
  },false);
close_savedata.addEventListener("click", function(){
    infoModal2.close();
},false);
close_savestanddata.addEventListener("click", function(){
    infoModal.close();
},false);
close_no1.addEventListener("click", function(){
    infoModal.close();
},false);
close_no2.addEventListener("click", function(){
    infoModal2.close();
},false);
