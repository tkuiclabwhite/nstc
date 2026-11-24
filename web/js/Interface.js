var ros = new ROSLIB.Ros({
  url: "ws://" + window.ROBOT_CONFIG.ip + ":9090"
});
ros.on('connection', function () {
  console.log('Connection made!');
  connectFlag = true;
  createTopics();
  resetfunction();
  document.body.style.backgroundImage = 'url(./picture/Background.jpg)';

  document.getElementById('resetButton').disabled = false;
  document.getElementById('connected').style.display = 'inline';
  document.getElementById('TorqueButton').disabled = false;
});
ros.on('error', function (error) {
  console.log('Error connecting to websocket server: ', error);
  document.getElementById('SaveButton').disabled = true;
  document.getElementById('ReadButton').disabled = true;
  document.getElementById('SaveStandButton').disabled = true;
  document.getElementById('ReadStandButton').disabled = true;
  document.getElementById('SendButton').disabled = true;
  document.getElementById('executeButton').disabled = true;
  document.getElementById('standButton').disabled = true;

  document.getElementById('MultipleButton').disabled = true;
  document.getElementById('MergeButton').disabled = true;
  document.getElementById('AddButton').disabled = true;
  document.getElementById('DeleteButton').disabled = true;
  document.getElementById('ReverseButton').disabled = true;
  document.getElementById('CopyButton').disabled = true;
  document.getElementById('CheckSumButton').disabled = true;
  document.getElementById('MoveUpButton').disabled = true;
  document.getElementById('MoveDownButton').disabled = true;
  document.getElementById('resetButton').disabled = true;
  document.getElementById('connected').style.display = 'none';
  document.getElementById('TorqueButton').disabled = true;
});
ros.on('close', function () {
  console.log('Connection to websocket server closed.');
  document.getElementById('SaveButton').disabled = true;
  document.getElementById('ReadButton').disabled = true;
  document.getElementById('SaveStandButton').disabled = true;
  document.getElementById('ReadStandButton').disabled = true;
  document.getElementById('SendButton').disabled = true;
  document.getElementById('executeButton').disabled = true;
  document.getElementById('standButton').disabled = true;

  document.getElementById('MultipleButton').disabled = true;
  document.getElementById('MergeButton').disabled = true;
  document.getElementById('AddButton').disabled = true;
  document.getElementById('DeleteButton').disabled = true;
  document.getElementById('ReverseButton').disabled = true;
  document.getElementById('CopyButton').disabled = true;
  document.getElementById('CheckSumButton').disabled = true;
  document.getElementById('MoveUpButton').disabled = true;
  document.getElementById('MoveDownButton').disabled = true;
  document.getElementById('resetButton').disabled = true;
  document.getElementById('connected').style.display = 'none';
  document.getElementById('TorqueButton').disabled = true;
});

var interface = new ROSLIB.Topic({
  ros: ros,
  name: '/package/InterfaceSend2Sector',
  messageType: 'nstc_msgs/InterfaceSend2Sector'
});

var SendPackage = new ROSLIB.Message({
  package: 0,
  sectorname: "",
  delay: [],
  cnt: 0
});


var SectorPackage = new ROSLIB.Topic({
  ros: ros,
  name: '/package/Sector',
  messageType: 'std_msgs/Int16'
});
var SendSectorPackage = new ROSLIB.Message({
  data : 0
});

var StandExecuteTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/package/StandExecute',
  messageType: 'std_msgs/Bool'
});

// 頭部直接走 driver_node 的 Head_Topic，邏輯 id 1/2/3 對應實體 28/29/30
var HeadTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/Head_Topic',
  messageType: 'nstc_msgs/HeadPackage'
});

var InterfaceSaveMotionData = new ROSLIB.Topic({
  ros: ros,
  name: '/package/InterfaceSaveMotion',
  messageType: 'nstc_msgs/SaveMotion'
});
var SaveMotionData = new ROSLIB.Message({
    name: "",
    motionstate: 0,
    id: 0,
    savestate: 0,
    saveflag: false,
    motionlist: [0],
    motordata: [0]
});

// ==========================================================
// [修正] 訂閱標準的 /joint_states
// ==========================================================
var JointStateListener = new ROSLIB.Topic({
  ros : ros,
  name : '/joint_states',           // 這裡要對應 Python 裡的 topic 名稱
  messageType : 'sensor_msgs/JointState' // 這是標準格式
});

JointStateListener.subscribe(function(message) {
  // 機器人回傳的格式是：
  // message.name = ["1", "2", "3"...] (馬達 ID)
  // message.position = [2048, 2048, ...] (馬達刻度)
  
  // 我們需要把這些亂序的資料，整理成我們函式看得懂的陣列 (Index 0 = ID 1)
  
  // 1. 先建立一個空陣列 (含頭部 28~30)。不填 0：沒收到的馬達保持 undefined，畫面就不會被歸零。
  //    /joint_states 還有 arm_ws MoveIt 假硬體在發（name 是 URDF 關節名、單位弧度），
  //    那種訊息一顆數字 ID 都沒有，以前會把整排刷成 0 造成閃爍。
  var formattedData = new Array(TOTAL_MOTORS);
  var found = 0;

  // 2. 把收到的資料填進去
  for(var i = 0; i < message.name.length; i++){
      var id = parseInt(message.name[i]); // 把 ID 字串 "1" 轉成數字 1
      var pos = Math.round(message.position[i]); // 確保刻度是整數

      // 防呆：確保 ID 在合理範圍內 (1~30)
      if(id >= 1 && id <= TOTAL_MOTORS) {
          formattedData[id - 1] = pos; // ID 1 存到 Index 0
          found++;
      }
  }
  if (found === 0) return; // 不是 driver_node 的訊息

  // 3. 呼叫更新函式
  Update_Motor_Input_Values(formattedData);
});
// ==============================

//-----
var SendPackageCallBack = null;
var ExecuteCallBack = null;

var connectFlag = false;
var myAddress = window.ROBOT_CONFIG.ip;

var executeSubscribeFlag = false;
var standSubscribeFlag = false;

var doSendFlag = false;
var doExecuteFlag = false;
var doStandFlag = false;

var FirstSend = true;

function createTopics()
{
  console.log("createtopics");
  console.log(SendPackageCallBack);

  if(SendPackageCallBack != null)
  {
    SendPackageCallBack.unsubscribe();
  }
  SendPackageCallBack = new ROSLIB.Topic({
    ros: ros,
    name: '/package/motioncallback',
    messageType: 'std_msgs/Bool'
  });
  SendPackageCallBack.subscribe(function(msg)
  {
    sleep(200);//wait for motionpackage 1000/60 = 166
    console.log("SendPackageCallBack");
    if(msg.data == true)
    {
      CheckSector(Number(document.getElementById('Sector').value));
    }
    else if(msg.data == false)
    {
      document.getElementById('label').innerHTML = "Send sector is fail !! Please try again !!";
    }
  });

  if(ExecuteCallBack != null)
  {
    ExecuteCallBack.unsubscribe();
  }
  ExecuteCallBack = new ROSLIB.Topic({
    ros: ros,
    name: '/package/executecallback',
    messageType: 'std_msgs/Bool'
  });
  ExecuteCallBack.subscribe(function (msg)
  {
    if(msg.data == true)
    {
      if(executeSubscribeFlag == true)
      {
        document.getElementById('label').innerHTML = "Execute is finish !!";
        document.getElementById('stand_label').innerHTML = "not standing";
        document.getElementById('SaveButton').disabled = false;
        document.getElementById('ReadButton').disabled = false;
        document.getElementById('SaveStandButton').disabled = false;
        document.getElementById('ReadStandButton').disabled = false;
        document.getElementById('SendButton').disabled = false;
        document.getElementById('executeButton').disabled = false;
        document.getElementById('standButton').disabled = false;
        
        document.getElementById('MultipleButton').disabled = false;
        document.getElementById('MergeButton').disabled = false;
        document.getElementById('AddButton').disabled = false;
        document.getElementById('DeleteButton').disabled = false;
        document.getElementById('ReverseButton').disabled = false;
        document.getElementById('CopyButton').disabled = false;
        document.getElementById('CheckSumButton').disabled = false;
        document.getElementById('MoveUpButton').disabled = false;
        document.getElementById('MoveDownButton').disabled = false;
        executeSubscribeFlag = false;
      }
      else if(standSubscribeFlag == true)
      {
        document.getElementById('stand_label').innerHTML = "is standing";
        document.getElementById('standButton').disabled = false;
        
        standSubscribeFlag = false;
      }
    }
    else
    {
      if(executeSubscribeFlag == true)
      {
        document.getElementById('label').innerHTML = "Execute is fail !! Please try again !!";
        executeSubscribeFlag = false;
      }
      else if(standSubscribeFlag == true)
      {
        document.getElementById('label').innerHTML = "Stand is fail !! Please try again !!";
        standSubscribeFlag = false;
      }
    }
  });
}

function enterAddress() {
  if (connectFlag) {
    ros.close();
    connectFlag = false;
  }

  console.log("connect is", connectFlag);
  myAddress = document.getElementById("addressSelect").value;
  console.log("Connecting address is", myAddress);

  const state = ros.socket ? ros.socket.readyState : WebSocket.CLOSED;
  if (state === WebSocket.OPEN || state === WebSocket.CONNECTING || state === WebSocket.CLOSING) {
    ros.close();
    console.log("state is", state);
    ros.once('close', () => ros.connect("ws://" + myAddress + ":9090"));
  } else {
    ros.connect("ws://" + myAddress + ":9090");
  }
  strategylocation();
}

var location_strategy = new ROSLIB.Topic({
  ros: ros,
  name: '/location',
  messageType: 'nstc_msgs/location'
});
var location_msg = new ROSLIB.Message({
  data: ""

});

function strategylocation() {
  // 获取选定的值
  location_msg.data = "/home/aa/Desktop/towen/src/strategy/strategy/"+ document.getElementById('strategy_location').value + "/Parameter";
  console.log(`Selected strategy location: ${location_msg.data}`);
  location_strategy.publish(location_msg);

}


function sleep(ms)
{
  var starttime = new Date().getTime();
  do{

  }while((new Date().getTime() - starttime) < ms)
}

function createTopicsDRC() {
  if(SensorPackage_Subscriber != null)
  {
    SensorPackage_Subscriber.unsubscribe();
  }
  var SensorPackage_Subscriber = new ROSLIB.Topic({
  ros: ros,
  name: '/package/sensorpackage',
  messageType: 'nstc_msgs/SensorPackage'
  });
  SensorPackage_Subscriber.subscribe(function (msg)
  {		
    document.getElementById("Roll").value = msg.x;
    document.getElementById("Pitch").value = msg.y;
    document.getElementById("Yaw").value = msg.z;
  });
  if(SendPackageCallBack != null)
  {
    SendPackageCallBack.unsubscribe();
  }
  SendPackageCallBack = new ROSLIB.Topic({
    ros: ros,
    name: '/package/motioncallback',
    messageType: 'std_msgs/Bool'
  });
  SendPackageCallBack.subscribe(function(msg)
  {
    sleep(200);//wait for motionpackage 1000/60 = 166
    console.log("SendPackageCallBack");
    if(msg.data == true)
    {
      CheckSector(Number(document.getElementById('Sector').value));
    }
    else if(msg.data == false)
    {
      document.getElementById('label').innerHTML = "Send sector is fail !! Please try again !!";
    }
  });

  if(ExecuteCallBack != null)
  {
    ExecuteCallBack.unsubscribe();
  }
  ExecuteCallBack = new ROSLIB.Topic({
    ros: ros,
    name: '/package/executecallback',
    messageType: 'std_msgs/Bool'
  });
  ExecuteCallBack.subscribe(function (msg)
  {
    if(msg.data == true)
    {
      if(executeSubscribeFlag == true)
      {
        executeSubscribeFlag = false;
        console.log("execute~")
      }
      else if(standSubscribeFlag == true)
      {
        standSubscribeFlag = false;
        console.log("stand~")
      }
    }
    else
    {
      if(executeSubscribeFlag == true)
      {
        executeSubscribeFlag = false;
        console.log("execute finish")
      }
      else if(standSubscribeFlag == true)
      {
        standSubscribeFlag = false;
        console.log("stand finish")
      }
    }
  });
}

function CheckSectorDRC(sectordata)
{
  console.log(sectordata)
  var LoadParameterClient = new ROSLIB.Service({
    ros : ros,
    name : '/package/InterfaceCheckSector',
    serviceType: 'nstc_msgs/CheckSector'
  });
  var parameter_request = new ROSLIB.ServiceRequest({
    data : sectordata
  });
  LoadParameterClient.callService(parameter_request , function(srv)
  {
    console.log("CheckSector");
    executeSubscribeFlag = false;
    standSubscribeFlag = false;
    if(srv.checkflag == true)
    {
      if(doSendFlag == true)
      {
        doSendFlag = false;
      }
      else if(doExecuteFlag == true)
      {
        SendSectorPackage.data = sectordata;
        SectorPackage.publish(SendSectorPackage);

        doExecuteFlag = false;
        executeSubscribeFlag = true;
      }
      else if(doStandFlag == true)
      {
        SendSectorPackage.data = sectordata;
        SectorPackage.publish(SendSectorPackage);
        
        doStandFlag = false;
        standSubscribeFlag = true;
      }
    }
    else
    {
      if(doSendFlag == true)
      {
        doSendFlag = false;
      }
      else if(doExecuteFlag == true)
      {
        doExecuteFlag = false;
      }
      else if(doStandFlag == true)
      {
        doStandFlag = false;
      }
    }
  });
}

function standDRC()
{
  doStandFlag = true;
  console.log("stand");

  CheckSector(29);
}

function executeDRC(motion)
{
  console.log("execute");
  doExecuteFlag = true;
  if(motion == "motion1")
  {
    CheckSectorDRC(Number(document.getElementById('Sector1').value));
    console.log("1")
    sleep(100)
  }
  else if(motion == "motion2")
  {
    CheckSectorDRC(Number(document.getElementById('Sector2').value));
    console.log("2")
    sleep(100)
  }
  else if(motion == "motion3")
  {
    CheckSectorDRC(Number(document.getElementById('Sector3').value));
    console.log("3")
    sleep(100)
  }
  
}

function CheckSector(sectordata)
{
  var LoadParameterClient = new ROSLIB.Service({
    ros : ros,
    name : '/package/InterfaceCheckSector',
    serviceType: 'nstc_msgs/CheckSector'
  });
  var parameter_request = new ROSLIB.ServiceRequest({
    data : sectordata
  });
  console.log(LoadParameterClient);
  console.log(sectordata);
  // SendSectorPackage.data = sectordata;
  // SectorPackage.publish(SendSectorPackage);

  LoadParameterClient.callService(parameter_request , function(srv)
  {
    console.log("CheckSector");
    console.log(srv.checkflag);
    console.log(doSendFlag);
    console.log(doExecuteFlag);

    executeSubscribeFlag = false;
    standSubscribeFlag = false;
    if(srv.checkflag == true)
    {
      if(doSendFlag == true)
      {
        document.getElementById('label').innerHTML = "Send sector is successful !!";
        document.getElementById('SaveButton').disabled = false;
        document.getElementById('ReadButton').disabled = false;
        document.getElementById('SaveStandButton').disabled = false;
        document.getElementById('ReadStandButton').disabled = false;
        document.getElementById('executeButton').disabled = false;
        document.getElementById('standButton').disabled = false;
        
        document.getElementById('MultipleButton').disabled = false;
        document.getElementById('MergeButton').disabled = false;
        document.getElementById('AddButton').disabled = false;
        document.getElementById('DeleteButton').disabled = false;
        document.getElementById('ReverseButton').disabled = false;
        document.getElementById('CopyButton').disabled = false;
        document.getElementById('CheckSumButton').disabled = false;
        document.getElementById('MoveUpButton').disabled = false;
        document.getElementById('MoveDownButton').disabled = false;
        doSendFlag = false;
      }
      else if(doExecuteFlag == true)
      {
        SendSectorPackage.data = sectordata;
        SectorPackage.publish(SendSectorPackage);

        doExecuteFlag = false;
        executeSubscribeFlag = true;
      }
      else if(doStandFlag == true)
      {
        console.log("stand");
        SendSectorPackage.data = sectordata;
        SectorPackage.publish(SendSectorPackage);
        
        doStandFlag = false;
        standSubscribeFlag = true;
      }
    }
    else
    {
      if(doSendFlag == true)
      {
        document.getElementById('label').innerHTML = "Sector is not correct !! Please try again !!";
        doSendFlag = false;
      }
      else if(doExecuteFlag == true)
      {
        console.log("execute");
        document.getElementById('label').innerHTML = "Sector is not correct !! Please check your sector file !!";
        document.getElementById('SaveButton').disabled = false;
        document.getElementById('ReadButton').disabled = false;
        document.getElementById('SaveStandButton').disabled = false;
        document.getElementById('ReadStandButton').disabled = false;
        document.getElementById('SendButton').disabled = false;
        document.getElementById('executeButton').disabled = false;
        document.getElementById('standButton').disabled = false;
        
        document.getElementById('MultipleButton').disabled = false;
        document.getElementById('MergeButton').disabled = false;
        document.getElementById('AddButton').disabled = false;
        document.getElementById('DeleteButton').disabled = false;
        document.getElementById('ReverseButton').disabled = false;
        document.getElementById('CopyButton').disabled = false;
        document.getElementById('CheckSumButton').disabled = false;
        document.getElementById('MoveUpButton').disabled = false;
        document.getElementById('MoveDownButton').disabled = false;
        doExecuteFlag = false;
      }
      else if(doStandFlag == true)
      {
        document.getElementById('label').innerHTML = "Sector is not correct !! Please check your sector file !!";
        document.getElementById('standButton').disabled = false;
        doStandFlag = false;
      }
    }
  });
}

function Save()
{
  // 1. 取得檔名
  const fileNameInput = document.getElementById('filename').value.trim();
  const fileName = fileNameInput !== '' ? fileNameInput : 'untitled';

  // 快速取得各組 DIV list
  const motionDivs    = document.getElementById('MotionTable'           ).getElementsByTagName('div');
  const relPosDivs    = document.getElementById('RelativePositionTable').getElementsByTagName('div');
  const relSpdDivs    = document.getElementById('RelativeSpeedTable'   ).getElementsByTagName('div');
  const absPosDivs    = document.getElementById('AbsolutePositionTable').getElementsByTagName('div');
  const absSpdDivs    = document.getElementById('AbsoluteSpeedTable'   ).getElementsByTagName('div');

  // Helper：建立 Message
  function makeMsg({ saveflag=false, savestate=0, motionstate=0, id=0 }) {
    return new ROSLIB.Message({
      name:        fileName,
      savestate:   savestate,
      saveflag:    saveflag,
      motionstate: motionstate,
      id:          id,
      motionlist:  [],
      motordata:   [],
      item_name:   ""  // [重點] 確保 Message 物件有這個欄位
    });
  }

  // =================================================================
  // 2. MotionTable (state=0)
  // =================================================================
  for (let i = 0; i < motionDivs.length; i += 2) {
    const id = Number(motionDivs[i].getElementsByClassName('textbox')[0].value) || 0;
    
    // [修正] 必須先取得 cells，才能讀取 cells[0] (Name)
    const cells = motionDivs[i+1].getElementsByClassName('textbox');
    const rowName = cells[0] ? cells[0].value : ""; 

    const msg = makeMsg({ savestate: 0, motionstate: 0, id: id });
    msg.item_name = rowName; 

    for (let j = 1; j <= 40; j++) {
      const v = Number(cells[j].value);
      msg.motionlist.push(isNaN(v) ? 0 : v);
    }
    InterfaceSaveMotionData.publish(msg);
  }

  // =================================================================
  // 3. Relative (pos state=1, spd state=2)
  // =================================================================
  for (let i = 0; i < relPosDivs.length; i += 2) {
    // --- Position ---
    const idPos = Number(relPosDivs[i].getElementsByClassName('textbox')[0].value) || 0;
    const posCells = relPosDivs[i+1].getElementsByClassName('textbox');
    const posName = posCells[0] ? posCells[0].value : ""; // 讀取名字

    const msgPos = makeMsg({ savestate: 0, motionstate: 1, id: idPos });
    msgPos.item_name = posName; // 存入名字

    for (let j = 1; j <= 27; j++) {
      const v = Number(posCells[j].value);
      msgPos.motordata.push(isNaN(v) ? 0 : v);
    }
    InterfaceSaveMotionData.publish(msgPos);

    // --- Speed ---
    const idSpd = Number(relSpdDivs[i].getElementsByClassName('textbox')[0].value) || 0;
    const spdCells = relSpdDivs[i+1].getElementsByClassName('textbox');
    // Speed 表格通常跟 Position 對應，名字也存一次無妨
    const spdName = spdCells[0] ? spdCells[0].value : "";

    const msgSpd = makeMsg({ savestate: 0, motionstate: 2, id: idSpd });
    msgSpd.item_name = spdName;

    for (let j = 1; j <= 27; j++) {
      const v = Number(spdCells[j].value);
      msgSpd.motordata.push(isNaN(v) ? 0 : v);
    }
    InterfaceSaveMotionData.publish(msgSpd);
  }

  // =================================================================
  // 4. Absolute (pos state=3, spd state=4)
  // =================================================================
  for (let i = 0; i < absPosDivs.length; i += 2) {
    // --- Position ---
    const idPosA = Number(absPosDivs[i].getElementsByClassName('textbox')[0].value) || 0;
    const posACells = absPosDivs[i+1].getElementsByClassName('textbox');
    const posAName = posACells[0] ? posACells[0].value : ""; // 讀取名字

    const msgPosA = makeMsg({ savestate: 0, motionstate: 3, id: idPosA });
    msgPosA.item_name = posAName; // 存入名字

    for (let j = 1; j <= 27; j++) {
      const v = Number(posACells[j].value);
      msgPosA.motordata.push(isNaN(v) ? 0 : v);
    }
    InterfaceSaveMotionData.publish(msgPosA);

    // --- Speed ---
    const idSpdA = Number(absSpdDivs[i].getElementsByClassName('textbox')[0].value) || 0;
    const spdACells = absSpdDivs[i+1].getElementsByClassName('textbox');
    const spdAName = spdACells[0] ? spdACells[0].value : "";

    const msgSpdA = makeMsg({ savestate: 0, motionstate: 4, id: idSpdA });
    msgSpdA.item_name = spdAName;

    for (let j = 1; j <= 27; j++) {
      const v = Number(spdACells[j].value);
      msgSpdA.motordata.push(isNaN(v) ? 0 : v);
    }
    InterfaceSaveMotionData.publish(msgSpdA);
  }

  // 5. 最後一筆：結束訊號
  const doneMsg = makeMsg({ saveflag: true, savestate: 0, motionstate: 0, id: 0 });
  InterfaceSaveMotionData.publish(doneMsg);

  // 6. UI 回饋
  document.getElementById('label').innerHTML = "Save file is successful !!";
}

function Read()
{
  var LoadParameterClient = new ROSLIB.Service({
    ros : ros,
    name : '/package/InterfaceReadSaveMotion',
    serviceType: 'nstc_msgs/ReadMotion'
  });
  var parameter_request = new ROSLIB.ServiceRequest({
    read : true,
    name : document.getElementById('filename').value,
    readstate : 0
  });
  LoadParameterClient.callService(parameter_request , function(MotionData)
  {
    var motionlistcnt = 0;
    var relativepositioncnt = 0;
    var relativespeedcnt = 0;
    var absolutepositioncnt = 0;
    var absolutespeedcnt = 0;
    
    console.log("Read Vector Count:", MotionData.vectorcnt);

    // [導師修正] 關鍵：在填寫前，先把所有表格清空！
    document.getElementById('MotionTable').innerHTML = "";
    document.getElementById('RelativePositionTable').innerHTML = "";
    document.getElementById('RelativeSpeedTable').innerHTML = "";
    document.getElementById('AbsolutePositionTable').innerHTML = "";
    document.getElementById('AbsoluteSpeedTable').innerHTML = "";

    for(var i = 0; i < MotionData.vectorcnt; i++)
    {
      var thisName = (MotionData.item_names && MotionData.item_names[i]) ? MotionData.item_names[i] : "";

      switch(MotionData.motionstate[i])
      {
        case 0: // MotionList
          NewMotionList();
          document.getElementById('MotionTable').getElementsByTagName('div')[motionlistcnt*2].getElementsByClassName('textbox')[0].value = MotionData.id[i];
          document.getElementById('MotionTable').getElementsByTagName('div')[motionlistcnt*2+1].getElementsByClassName('textbox')[0].value = thisName;
          
          for(var j = 0; j < 40; j++)
          {
            document.getElementById('MotionTable').getElementsByTagName('div')[motionlistcnt*2+1].getElementsByClassName('textbox')[j+1].value = MotionData.motionlist[motionlistcnt*40+j];
          }
          motionlistcnt++;
          break;

        case 1: // Relative Position
          NewRelativePosition();
          document.getElementById('RelativePositionTable').getElementsByTagName('div')[relativepositioncnt*2].getElementsByClassName('textbox')[0].value = MotionData.id[i];
          document.getElementById('RelativePositionTable').getElementsByTagName('div')[relativepositioncnt*2+1].getElementsByClassName('textbox')[0].value = thisName;

          for(var j = 0; j < 27; j++)
          {
            document.getElementById('RelativePositionTable').getElementsByTagName('div')[relativepositioncnt*2+1].getElementsByClassName('textbox')[j+1].value = MotionData.relativedata[relativepositioncnt*27+relativespeedcnt*27+j];
          }
          relativepositioncnt++;
          break;

        case 2: // Relative Speed
          NewRelativeSpeed();
          document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[relativespeedcnt*2].getElementsByClassName('textbox')[0].value = MotionData.id[i];
          document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[relativespeedcnt*2+1].getElementsByClassName('textbox')[0].value = thisName;

          for(var j = 0; j < 27; j++)
          {
            document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[relativespeedcnt*2+1].getElementsByClassName('textbox')[j+1].value = MotionData.relativedata[relativepositioncnt*27+relativespeedcnt*27+j];
          }
          relativespeedcnt++;
          break;

        case 3: // Absolute Position
          NewAbsolutePosition();
          document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[absolutepositioncnt*2].getElementsByClassName('textbox')[0].value = MotionData.id[i];
          document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[absolutepositioncnt*2+1].getElementsByClassName('textbox')[0].value = thisName;

          for(var j = 0; j < 27; j++)
          {
            document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[absolutepositioncnt*2+1].getElementsByClassName('textbox')[j+1].value = MotionData.absolutedata[absolutepositioncnt*27+absolutespeedcnt*27+j];
          }
          absolutepositioncnt++;
          break;

        case 4: // Absolute Speed
          NewAbsoluteSpeed();
          document.getElementById('AbsoluteSpeedTable').getElementsByTagName('div')[absolutespeedcnt*2].getElementsByClassName('textbox')[0].value = MotionData.id[i];
          document.getElementById('AbsoluteSpeedTable').getElementsByTagName('div')[absolutespeedcnt*2+1].getElementsByClassName('textbox')[0].value = thisName;

          for(var j = 0; j < 27; j++)
          {
            document.getElementById('AbsoluteSpeedTable').getElementsByTagName('div')[absolutespeedcnt*2+1].getElementsByClassName('textbox')[j+1].value = MotionData.absolutedata[absolutepositioncnt*27+absolutespeedcnt*27+j];
          }
          absolutespeedcnt++;
          break;
      }
    }
    document.getElementById('label').innerHTML = "Read file is successful !!";
  });
}

function SaveStand() {
  if (document.getElementById('Lockedstand').checked) {
    alert("Locked Stand is enabled. Uncheck Locked Stand to save.");
    return;
  }
  // [修改] 強制檔名為 "stand"，無視輸入框的內容
  const fileName = "stand";

  // 取得各組 DIV list
  const motionDivs    = document.getElementById('MotionTable'           ).getElementsByTagName('div');
  const relPosDivs    = document.getElementById('RelativePositionTable').getElementsByTagName('div');
  const relSpdDivs    = document.getElementById('RelativeSpeedTable'   ).getElementsByTagName('div');
  const absPosDivs    = document.getElementById('AbsolutePositionTable').getElementsByTagName('div');
  const absSpdDivs    = document.getElementById('AbsoluteSpeedTable'   ).getElementsByTagName('div');

  // Helper
  function makeMsg({ saveflag=false, savestate=1, motionstate=0, id=0 }) {
    return new ROSLIB.Message({
      name:        fileName, // 這裡會自動用 "stand"
      savestate:   savestate,
      saveflag:    saveflag,
      motionstate: motionstate,
      id:          id,
      motionlist:  [],   
      motordata:   [],
      item_name:   ""    
    });
  }

  // 2. MotionTable
  for (let i = 0; i < motionDivs.length; i += 2) {
    const id = 29; // 強制 ID 29
    
    const cells = motionDivs[i+1].getElementsByClassName('textbox');
    const rowName = cells[0] ? cells[0].value : ""; 
    const msg = makeMsg({ savestate: 1, motionstate: 0, id: id });
    msg.item_name = rowName;

    for (let j = 1; j <= 40; j++) {
      const v = Number(cells[j].value);
      msg.motionlist.push(isNaN(v) ? 0 : v);
    }
    InterfaceSaveMotionData.publish(msg);
  }

  // 3. Relative (Pos & Spd)
  for (let i = 0; i < relPosDivs.length; i += 2) {
    const id = 29; // 強制 ID 29

    // Position
    const posCells = relPosDivs[i+1].getElementsByClassName('textbox');
    const posName = posCells[0] ? posCells[0].value : "";
    const msgPos = makeMsg({ savestate: 1, motionstate: 1, id: id });
    msgPos.item_name = posName;
    for (let j = 1; j <= 27; j++) {
      const v = Number(posCells[j].value);
      msgPos.motordata.push(isNaN(v) ? 0 : v);
    }
    InterfaceSaveMotionData.publish(msgPos);

    // Speed
    const spdCells = relSpdDivs[i+1].getElementsByClassName('textbox');
    const spdName = spdCells[0] ? spdCells[0].value : "";
    const msgSpd = makeMsg({ savestate: 1, motionstate: 2, id: id });
    msgSpd.item_name = spdName;
    for (let j = 1; j <= 27; j++) {
      const v = Number(spdCells[j].value);
      msgSpd.motordata.push(isNaN(v) ? 0 : v);
    }
    InterfaceSaveMotionData.publish(msgSpd);
  }

  // 4. Absolute (Pos & Spd)
  for (let i = 0; i < absPosDivs.length; i += 2) {
    const id = 29; // 強制 ID 29

    // Position
    const posACells = absPosDivs[i+1].getElementsByClassName('textbox');
    const posAName = posACells[0] ? posACells[0].value : "";
    const msgPosA = makeMsg({ savestate: 1, motionstate: 3, id: id });
    msgPosA.item_name = posAName;
    for (let j = 1; j <= 27; j++) {
      const v = Number(posACells[j].value);
      msgPosA.motordata.push(isNaN(v) ? 0 : v);
    }
    InterfaceSaveMotionData.publish(msgPosA);

    // Speed
    const spdACells = absSpdDivs[i+1].getElementsByClassName('textbox');
    const spdAName = spdACells[0] ? spdACells[0].value : "";
    const msgSpdA = makeMsg({ savestate: 1, motionstate: 4, id: id });
    msgSpdA.item_name = spdAName;
    for (let j = 1; j <= 27; j++) {
      const v = Number(spdACells[j].value);
      msgSpdA.motordata.push(isNaN(v) ? 0 : v);
    }
    InterfaceSaveMotionData.publish(msgSpdA);
  }

  // 5. 結束訊號
  const doneMsg = makeMsg({ saveflag: true, savestate: 1, motionstate: 0, id: 0 });
  InterfaceSaveMotionData.publish(doneMsg);

  document.getElementById('label').innerHTML = "SaveStand successful (Saved as 'stand.ini') !!";
}

function ReadStand()
{
  if (document.getElementById('Lockedstand').checked) {
    alert("Locked Stand is enabled. Uncheck Locked Stand to read.");
    return;
  }
  console.log("ReadStand (Force 'stand')");
  var LoadParameterClient = new ROSLIB.Service({
    ros : ros,
    name : '/package/InterfaceReadSaveMotion',
    serviceType: 'nstc_msgs/ReadMotion'
  });
  
  // [修改] 強制讀取 "stand"
  var filename = "stand";

  var parameter_request = new ROSLIB.ServiceRequest({
    read : true,
    name : filename,
    readstate : 1 
  });
  
  console.log("Reading file: " + filename);

  LoadParameterClient.callService(parameter_request , function(MotionData){
    // =================================================================
    document.getElementById('MotionTable').innerHTML = "";
    document.getElementById('RelativePositionTable').innerHTML = "";
    document.getElementById('RelativeSpeedTable').innerHTML = "";
    document.getElementById('AbsolutePositionTable').innerHTML = "";
    document.getElementById('AbsoluteSpeedTable').innerHTML = "";
    // =================================================================
    
    var motionlistcnt = 0;
    var relativepositioncnt = 0;
    var relativespeedcnt = 0;
    var absolutepositioncnt = 0;
    var absolutespeedcnt = 0;
    
    if (MotionData.readcheck == false)
    {
      document.getElementById('label').innerHTML = "ReadStand file is fail !! Check if 'stand.ini' exists.";
      return;
    }
    else if (MotionData.readcheck == true)
    {
      console.log("ReadStand successful !!");
      
      for(var i = 0; i < MotionData.vectorcnt; i++)
      {
        var thisName = (MotionData.item_names && MotionData.item_names[i]) ? MotionData.item_names[i] : "";

        switch(MotionData.motionstate[i])
        {
          case 0: // MotionList
            NewMotionList();
            document.getElementById('MotionTable').getElementsByTagName('div')[motionlistcnt*2].getElementsByClassName('textbox')[0].value = MotionData.id[i];
            document.getElementById('MotionTable').getElementsByTagName('div')[motionlistcnt*2+1].getElementsByClassName('textbox')[0].value = thisName;
            for(var j = 0; j < 40; j++) {
              document.getElementById('MotionTable').getElementsByTagName('div')[motionlistcnt*2+1].getElementsByClassName('textbox')[j+1].value = MotionData.motionlist[motionlistcnt*40+j];
            }
            motionlistcnt++;
            break;

          case 1: // Relative Position
            NewRelativePosition();
            document.getElementById('RelativePositionTable').getElementsByTagName('div')[relativepositioncnt*2].getElementsByClassName('textbox')[0].value = MotionData.id[i];
            document.getElementById('RelativePositionTable').getElementsByTagName('div')[relativepositioncnt*2+1].getElementsByClassName('textbox')[0].value = thisName;
            for(var j = 0; j < 27; j++) {
              document.getElementById('RelativePositionTable').getElementsByTagName('div')[relativepositioncnt*2+1].getElementsByClassName('textbox')[j+1].value = MotionData.relativedata[relativepositioncnt*27+relativespeedcnt*27+j];
            }
            relativepositioncnt++;
            break;

          case 2: // Relative Speed
            NewRelativeSpeed(); 
            document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[relativespeedcnt*2].getElementsByClassName('textbox')[0].value = MotionData.id[i];
            document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[relativespeedcnt*2+1].getElementsByClassName('textbox')[0].value = thisName;
            for(var j = 0; j < 27; j++) {
              document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[relativespeedcnt*2+1].getElementsByClassName('textbox')[j+1].value = MotionData.relativedata[relativepositioncnt*27+relativespeedcnt*27+j];
            }
            relativespeedcnt++;
            break;

          case 3: // Absolute Position
            NewAbsolutePosition();
            document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[absolutepositioncnt*2].getElementsByClassName('textbox')[0].value = MotionData.id[i];
            document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[absolutepositioncnt*2+1].getElementsByClassName('textbox')[0].value = thisName;
            for(var j = 0; j < 27; j++) {
              document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[absolutepositioncnt*2+1].getElementsByClassName('textbox')[j+1].value = MotionData.absolutedata[absolutepositioncnt*27+absolutespeedcnt*27+j];
            }
            absolutepositioncnt++;
            break;

          case 4: // Absolute Speed
            NewAbsoluteSpeed();
            document.getElementById('AbsoluteSpeedTable').getElementsByTagName('div')[absolutespeedcnt*2].getElementsByClassName('textbox')[0].value = MotionData.id[i];
            document.getElementById('AbsoluteSpeedTable').getElementsByTagName('div')[absolutespeedcnt*2+1].getElementsByClassName('textbox')[0].value = thisName;
            for(var j = 0; j < 27; j++) {
              document.getElementById('AbsoluteSpeedTable').getElementsByTagName('div')[absolutespeedcnt*2+1].getElementsByClassName('textbox')[j+1].value = MotionData.absolutedata[absolutepositioncnt*27+absolutespeedcnt*27+j];
            }
            absolutespeedcnt++;
            break;
        }
      }
      document.getElementById('label').innerHTML = "ReadStand file is successful !!";
    }
  });
}

function Send() {
    doSendFlag = true;
    // --- UI 控制 (保持原本樣式) ---
    document.getElementById('label').innerHTML = "";
    document.getElementById('SaveButton').disabled = true;
    document.getElementById('ReadButton').disabled = true;
    document.getElementById('SaveStandButton').disabled = true;
    document.getElementById('ReadStandButton').disabled = true;
    document.getElementById('executeButton').disabled = true;
    document.getElementById('standButton').disabled = true;
    document.getElementById('MultipleButton').disabled = true;
    document.getElementById('MergeButton').disabled = true;
    document.getElementById('AddButton').disabled = true;
    document.getElementById('DeleteButton').disabled = true;
    document.getElementById('ReverseButton').disabled = true;
    document.getElementById('CopyButton').disabled = true;
    document.getElementById('CheckSumButton').disabled = true;
    document.getElementById('MoveUpButton').disabled = true;
    document.getElementById('MoveDownButton').disabled = true;

    var MotionList = [];
    var ID = Number(document.getElementById('SendID').value);
    console.log("id = ", ID);
    var Sector = Number(document.getElementById('Sector').value);
    console.log("Sector = ", Sector);

    // 基本防呆
    if (document.getElementById('Locked29').checked && Sector == 29) {
        alert("Sector 29 is Locked. Please try again. ");
        resetfunction();
        return;
    } else if (Sector < 1) {
        alert("Sector is not find. Please try again. ");
        resetfunction();
        return;
    }

    // =======================================================================
    // [輔助函式] 自動發送子動作 (已修正為一次整包發送)
    // =======================================================================
    function SendSubSector(targetID) {
        // 1. 搜尋 Absolute Table
        var absDivs = document.getElementById('AbsolutePositionTable').getElementsByTagName('div');
        for (let i = 0; i < absDivs.length; i += 2) {
            if (i >= absDivs.length) break;
            var idBox = absDivs[i].getElementsByClassName('textbox')[0];
            if (idBox && Number(idBox.value) === targetID) {
                console.log("Found in Absolute Table: ", targetID);
                
                var subList = [83, 84];
                var subOpcode = document.getElementById('Lockedstand').checked ? 242 : 241;
                subList.push(subOpcode);

                // 抓取 27 顆馬達數據
                for (let j = 0; j < 27; j++) {
                    var speed = Number(document.getElementById('AbsoluteSpeedTable').getElementsByTagName('div')[i + 1].getElementsByClassName('textbox')[j + 1].value) || 0;
                    var pos = Number(document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[i + 1].getElementsByClassName('textbox')[j + 1].value) || 0;
                    subList.push(speed, pos);
                }
                subList.push(78, 69);

                // ★ 修正重點：一次打包發送 ★
                SendPackage.sectorname = String(targetID);
                SendPackage.package = subList; 
                interface.publish(SendPackage);
                
                return; // 找到就結束
            }
        }

        // 2. 搜尋 Relative Table
        var relDivs = document.getElementById('RelativePositionTable').getElementsByTagName('div');
        for (let i = 0; i < relDivs.length; i += 2) {
            if (i >= relDivs.length) break;
            var idBox = relDivs[i].getElementsByClassName('textbox')[0];
            if (idBox && Number(idBox.value) === targetID) {
                console.log("Found in Relative Table: ", targetID);

                var subList = [83, 84, 243]; // Relative Opcode

                // 抓取 27 顆馬達數據
                for (let j = 0; j < 27; j++) {
                    var speed = Number(document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[i + 1].getElementsByClassName('textbox')[j + 1].value) || 0;
                    var pos = Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[i + 1].getElementsByClassName('textbox')[j + 1].value) || 0;
                    subList.push(speed, pos);
                }
                subList.push(78, 69);

                // ★ 修正重點：一次打包發送 ★
                SendPackage.sectorname = String(targetID);
                SendPackage.package = subList;
                interface.publish(SendPackage);

                return;
            }
        }
        console.log("ID " + targetID + " not found in tables.");
    }

    // =======================================================================
    // 主發送邏輯
    // =======================================================================

    // --- 情況 1: Absolute Mode ---
    if (document.getElementById("AbsolutePosition").style.display == "initial" ||
        document.getElementById("AbsoluteSpeed").style.display == "initial") {
        
        console.log("Detected Mode: Absolute");
        
        var absDivs = document.getElementById('AbsolutePositionTable').getElementsByTagName('div');
        for (let i = 0; i < absDivs.length; i++) {
            var row = absDivs[i];
            if (!row) continue;
            var idBox = row.getElementsByClassName('textbox')[0];
            const rowId = idBox ? Number(idBox.value) : -1;

            if (ID === rowId) {
                // 建構 Header
                MotionList.push(83, 84);
                
                const opcode = document.getElementById('Lockedstand').checked ? 242 : 241;
                MotionList.push(opcode);

                // 抓取數據
                for (let j = 0; j < 27; j++) {
                    const speed = Number(document.getElementById('AbsoluteSpeedTable').getElementsByTagName('div')[i + 1].getElementsByClassName('textbox')[j + 1].value) || 0;
                    const pos = Number(document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[i + 1].getElementsByClassName('textbox')[j + 1].value) || 0;
                    MotionList.push(speed, pos);
                }
                
                // 建構 Footer
                MotionList.push(78, 69);

                // ★ 修正重點：一次打包發送 ★
                SendPackage.sectorname = document.getElementById('Sector').value;
                SendPackage.package = MotionList;
                interface.publish(SendPackage);
                
                break;
            }
        }
    }

    // --- 情況 2: Relative Mode ---
    else if (document.getElementById("RelativePosition").style.display == "initial" ||
             document.getElementById("RelativeSpeed").style.display == "initial") {
        
        console.log("Detected Mode: Relative");
        
        var relDivs = document.getElementById('RelativePositionTable').getElementsByTagName('div');
        for (let i = 0; i < relDivs.length; i += 2) {
            var row = relDivs[i];
            if (!row) continue;
            var idBox = row.getElementsByClassName('textbox')[0];
            const rowId = idBox ? Number(idBox.value) : -1;

            if (ID === rowId) {
                MotionList.push(83, 84, 243);

                for (let j = 0; j < 27; j++) {
                    const speed = Number(document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[i + 1].getElementsByClassName('textbox')[j + 1].value) || 0;
                    const pos = Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[i + 1].getElementsByClassName('textbox')[j + 1].value) || 0;
                    MotionList.push(speed, pos);
                }

                MotionList.push(78, 69);

                // ★ 修正重點：一次打包發送 ★
                SendPackage.sectorname = document.getElementById('Sector').value;
                SendPackage.package = MotionList;
                interface.publish(SendPackage);
                
                break;
            }
        }
    }

    // --- 情況 3: MotionList Mode ---
    else if (document.getElementById("MotionList").style.display == "initial") {
        
        console.log("Detected Mode: MotionList (Opcode 244)");
        const table = document.getElementById('MotionTable');
        const rows = table.getElementsByTagName('div');

        for (let i = 0; i < rows.length - 1; i += 2) {
            const idBox = rows[i].getElementsByClassName('textbox')[0];
            if (Number(idBox.value) !== ID) continue;

            const boxes = rows[i + 1].getElementsByClassName('textbox');

            // 1. [依賴檢查] 先把用到的子動作都送一遍
            for (let m = 0; m < 40; m++) {
                // 表格結構：奇數索引是 ID (A1, A2...)
                const sub_id = Number(boxes[2 * m + 1]?.value) || 0;
                if (sub_id !== 0) {
                    SendSubSector(sub_id);
                    // 為了確保 Python 端有時間處理子動作載入，這裡可以用極短的等待(非必要，視情況而定)
                    // 但因為是一次性封包，處理速度應該很快，通常不需要 sleep
                }
            }

            // 2. 建構 MotionList 本體封包
            MotionList.push(83, 84, 244);
            
            // 抓取列表數據
            for (let j = 1; j <= 40; j++) {
                MotionList.push(Number(boxes[j].value) || 0);
            }
            
            MotionList.push(78, 69);

            // ★ 修正重點：一次打包發送 ★
            SendPackage.sectorname = document.getElementById('Sector').value;
            SendPackage.package = MotionList;
            interface.publish(SendPackage);

            // 處理 Delay 和 cnt (這是修改 InterfaceSend2Sector 的其他欄位，這部分保持不變)
            for (let m = 0; m < 40; m++) {
                const motion_delay = Number(boxes[2 * m + 2]?.value) || 0;
                if (motion_delay) {
                    SendPackage.delay[m] = motion_delay;
                    SendPackage.cnt = m;
                }
            }
            break;
        }
    }

    // 清理
    MotionList = [];
}

function MotionSaveId(motion_id) {
  console.log("motion_id =", motion_id);
  SendPackage.sectorname = String(motion_id);

  // 1) 一次性获取表格和所有行
  const speedRows = document
    .getElementById('RelativeSpeedTable')
    .getElementsByTagName('div');
  const posRows = document
    .getElementById('RelativePositionTable')
    .getElementsByTagName('div');

  // 2) 初始 header
  const MotionList = [83, 84, 245];

  // 3) 找到对应 row
  for (let i = 0; i < posRows.length; i += 2) {
    const idTextbox = posRows[i].getElementsByClassName('textbox')[0];
    if (Number(idTextbox.value) !== motion_id) continue;

    // 4) 拿到这一行 speed / pos 的 textbox 集合
    const speedBoxes = speedRows[i + 1].getElementsByClassName('textbox');
    const posBoxes   = posRows[i + 1].getElementsByClassName('textbox');

    // 5) 一次性 push speed,pos 序列
    for (let j = 1; j <= 27; j++) {
      MotionList.push(
        Number(speedBoxes[j].value),
        Number(posBoxes[j].value)
      );
    }

    // 6) 加尾标
    MotionList.push(78, 69);

    break;  // 找到就跳出
  }

  // 7) 发布
  console.log("245 publish start, length =", MotionList.length);
  MotionList.forEach(b => {
    SendPackage.package = b;
    interface.publish(SendPackage);
    console.log(b);
    sleep(2);
  });
  console.log("245 publish end");
}

function Locked()
{
  if (!document.getElementById('Locked29').checked)
  {
    document.getElementById('label').innerHTML = "Sector 29 is Unlocked";
  }
  else if (document.getElementById('Locked29').checked)
  {
    document.getElementById('label').innerHTML = "Sector 29 is Locked";
  }
}

function Locked2()
{
  if (!document.getElementById('Lockedstand').checked)
  {
    document.getElementById('label').innerHTML = "Stand Unlocked";
  }
  else if (document.getElementById('Lockedstand').checked)
  {
    document.getElementById('label').innerHTML = "Stand Locked";
  }
}

function execute()
{
  doExecuteFlag = true;
  document.getElementById('label').innerHTML = "";
  document.getElementById('SaveButton').disabled = true;
  document.getElementById('ReadButton').disabled = true;
  document.getElementById('SaveStandButton').disabled = true;
  document.getElementById('ReadStandButton').disabled = true;
  document.getElementById('SendButton').disabled = true;
  document.getElementById('executeButton').disabled = true;
  document.getElementById('standButton').disabled = true;
  
  document.getElementById('MultipleButton').disabled = true;
  document.getElementById('MergeButton').disabled = true;
  document.getElementById('AddButton').disabled = true;
  document.getElementById('DeleteButton').disabled = true;
  document.getElementById('ReverseButton').disabled = true;
  document.getElementById('CopyButton').disabled = true;
  document.getElementById('CheckSumButton').disabled = true;
  document.getElementById('MoveUpButton').disabled = true;
  document.getElementById('MoveDownButton').disabled = true;
  console.log("23232323232323")
  CheckSector(Number(document.getElementById('Sector').value));
}

function stand()
{
  document.getElementById('label').innerHTML = "";
  document.getElementById('standButton').disabled = true;
  standSubscribeFlag = true;
  StandExecuteTopic.publish(new ROSLIB.Message({ data: true }));
}


// 頭部三顆回到 2048。speed 不能給 0，driver 寫入 Profile Velocity 時 0 代表全速。
const HEAD_CENTER_POS = 2048;
const HEAD_CENTER_SPEED = 50;
function HeadCenter()
{
  for (let id = 1; id <= 3; id++) {
    HeadTopic.publish(new ROSLIB.Message({ id: id, position: HEAD_CENTER_POS, speed: HEAD_CENTER_SPEED }));
  }
  document.getElementById('label').innerHTML = "Head -> " + HEAD_CENTER_POS;
}

function resetfunction()
{
  document.getElementById('label').innerHTML = "";
  document.getElementById('SaveButton').disabled = false;
  document.getElementById('ReadButton').disabled = false;
  document.getElementById('SaveStandButton').disabled = false;
  document.getElementById('ReadStandButton').disabled = false;
  document.getElementById('SendButton').disabled = false;
  document.getElementById('executeButton').disabled = false;
  document.getElementById('standButton').disabled = false;

  document.getElementById('MultipleButton').disabled = false;
  document.getElementById('MergeButton').disabled = false;
  document.getElementById('AddButton').disabled = false;
  document.getElementById('DeleteButton').disabled = false;
  document.getElementById('ReverseButton').disabled = false;
  document.getElementById('CopyButton').disabled = false;
  document.getElementById('CheckSumButton').disabled = false;
  document.getElementById('MoveUpButton').disabled = false;
  document.getElementById('MoveDownButton').disabled = false;
}

function addreduce(value)
{
  var addreduce = value;
  console.log(addreduce);
  var chooseID=document.getElementById("chooseID").value;
  var resetID=document.getElementById("resetID").value;
  var n=0;
  var numflag=false;
  var Motorflag = false;
  if(document.getElementById("RelativePosition").style.display == "initial")
  {
    console.log("RelativePosition");
    for(var i = 0; i < document.getElementById('RelativePositionTable').getElementsByTagName('div').length; i += 2)
	  {
      if(document.getElementById('RelativePositionTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[0].value == chooseID)
	    {
        n = i;
        numflag = true;
        break;
      }  
    }
    if(resetID>27 || resetID<1)
    {
      numflag = false;
      Motorflag = true;
    }
    if(numflag == true)
	  {
      switch(addreduce)
      {
        case "add_5":
          var value = Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value);
          document.getElementById('RelativePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value = value + 5;
          break;
        case "add_10":
          var value = Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value);
          document.getElementById('RelativePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value = value + 10;
          break;
        case "add_100":
          var value = Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value);
          document.getElementById('RelativePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value = value + 100;
          break;
        case "reduce_5":
          var value = Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value);
          document.getElementById('RelativePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value = value - 5;
          break;
        case "reduce_10":
          var value = Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value);
          document.getElementById('RelativePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value = value - 10;
          break;
        case "reduce_100":
          var value = Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value);
          document.getElementById('RelativePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value = value - 100;
          break;
        case "resetmotor":
          var value = Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value);
          document.getElementById('RelativePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value = 2048;
          break;
        default:
          var getvalue = Number(document.getElementById('addvalue').value);
          var value = Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value);
          document.getElementById('RelativePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value = getvalue + value;
          break;
      }
      document.getElementById('label').innerHTML = "add & reduce is successful !!";
    }
    else
	  {
      if(Motorflag)
      {
        document.getElementById('label').innerHTML = "No this MotorID !!";
      }
      else
      {
        document.getElementById('label').innerHTML = "add & reduce is fail !! No this ID !!";
      }
    }
  }
  else if(document.getElementById("AbsolutePosition").style.display == "initial") 
  {
    console.log("AbsolutePositionTable");
    for(var i = 0; i < document.getElementById('AbsolutePositionTable').getElementsByTagName('div').length; i += 2)
	  {
      if(document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[0].value == chooseID)
	    {
        n = i;
        numflag = true;
        break;
      }  
    }
    if(resetID>27 || resetID<1)
    {
      numflag = false;
      Motorflag = true;
    }
    if(numflag == true)
	  {
      switch(addreduce)
      {
        case "add_5":
          var value = Number(document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value);
          document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value = value + 5;
          break;
        case "add_10":
          var value = Number(document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value);
          document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value = value + 10;
          break;
        case "add_100":
          var value = Number(document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value);
          document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value = value + 100;
          break;
        case "reduce_5":
          var value = Number(document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value);
          document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value = value - 5;
          break;
        case "reduce_10":
          var value = Number(document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value);
          document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value = value - 10;
          break;
        case "reduce_100":
          var value = Number(document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value);
          document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value = value - 100;
          break;
        case "resetmotor":
          var value = Number(document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value);
          document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value = 2048;
          break;
        default:
          var getvalue = Number(document.getElementById('addvalue').value);
          var value = Number(document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value);
          document.getElementById('AbsolutePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[resetID].value = getvalue + value;
          break;
      }
      document.getElementById('label').innerHTML = "add & reduce is successful !!";
    }
    else
	  {
      if(Motorflag)
      {
        document.getElementById('label').innerHTML = "No this MotorID !!";
      }
      else
      {
        document.getElementById('label').innerHTML = "add & reduce is fail !! No this ID !!";
      }
    }
  }
}

function Multiple()
{
  var num=document.getElementById("chose_multiple").value;
  var times=document.getElementById("times").value;
  var n=0;
  var numflag=false;
  if(document.getElementById("MotionList").style.display == "initial")
  {
   
  }
  else if(document.getElementById("RelativePosition").style.display == "initial")
  {
    for(var i = 0; i < document.getElementById('RelativePositionTable').getElementsByTagName('div').length; i += 2)
	  {
      if(document.getElementById('RelativePositionTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[0].value == num)
	    {
        n = i;
        numflag = true;
        break;
      }  
    }
    if(numflag == true)
	  {
      for (var j = 1; j <= 27; j++)
	    {
        var value = Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[j].value);
        document.getElementById('RelativePositionTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[j].value = value * times;
      }
      document.getElementById('label').innerHTML = "Multiple is successful !!";
    }
    else
	  {
      document.getElementById('label').innerHTML = "Multiple is fail !! No this ID !!";
    }
  
  } 
  else if(document.getElementById("RelativeSpeed").style.display == "initial")
  {
    for(var i = 0; i<document.getElementById('RelativeSpeedTable').getElementsByTagName('div').length; i += 2)
	  {
      if(document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[0].value == num)
	    {
        n = i;
        numflag = true;
        break;
      }  
    }
    if(numflag==true)
	  {
      for (var j = 1; j <= 27; j++)
	    {
        var value = Number(document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[j].value);
        document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[n+1].getElementsByClassName('textbox')[j].value = value * times;
      }
      document.getElementById('label').innerHTML = "Multiple is successful !!";
    }
    else
	  {
      document.getElementById('label').innerHTML = "Multiple is fail !! No this ID !!";
    }
  }  
  else if(document.getElementById("AbsolutePosition").style.display == "initial")
  {

  }
  else if(document.getElementById("AbsoluteSpeed").style.display ==  "initial")
  {

  }
}

function Merge(){
  var num1 = document.getElementById("Merge1").value;
  var num2 = document.getElementById("Merge2").value;
  var num1flag=false;
  var num2flag=false;
  var n1=0;
  var n2=0;
  if(document.getElementById("MotionList").style.display == "initial")
  {
   
  }
  else if(document.getElementById("RelativePosition").style.display == "initial" || document.getElementById("RelativeSpeed").style.display ==  "initial")
  {
    for(var i = 0;i < document.getElementById('RelativePositionTable').getElementsByTagName('div').length && num1 != num2; i += 2)
	  {
      if(document.getElementById('RelativePositionTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[0].value == num1)
	    {
        n1 = i;
        num1flag = true;
      }
      if(document.getElementById('RelativePositionTable').getElementsByTagName('div')[i].getElementsByClassName('textbox')[0].value == num2)
	    {
        n2 = i;
        num2flag = true;
      }
      if(num1flag == true && num2flag == true)
      {
      break;
      }
    }
    if(num1flag == true && num2flag == true)
	  {
      for(var j = 1; j <= 27; j++)
	    {
        var pos1 = Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[n1+1].getElementsByClassName('textbox')[j].value);
        var pos2 = Number(document.getElementById('RelativePositionTable').getElementsByTagName('div')[n2+1].getElementsByClassName('textbox')[j].value);
        document.getElementById('RelativePositionTable').getElementsByTagName('div')[n2+1].getElementsByClassName('textbox')[j].value = pos1 + pos2;
        var spd1 = Number(document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[n1+1].getElementsByClassName('textbox')[j].value);
        var spd2 = Number(document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[n2+1].getElementsByClassName('textbox')[j].value);
        document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[n2+1].getElementsByClassName('textbox')[j].value = Math.max(spd1, spd2);
      }
      document.getElementById('RelativePositionTable').removeChild(document.getElementById('RelativePositionTable').getElementsByTagName('div')[n1]);
      document.getElementById('RelativePositionTable').removeChild(document.getElementById('RelativePositionTable').getElementsByTagName('div')[n1]);
      document.getElementById('RelativeSpeedTable').removeChild(document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[n1]);
      document.getElementById('RelativeSpeedTable').removeChild(document.getElementById('RelativeSpeedTable').getElementsByTagName('div')[n1]);
      document.getElementById('label').innerHTML = "Merge is successful !!";
    }
    else
	  {
      if(num1 == num2)
      {
        document.getElementById('label').innerHTML = "ID1 can't be the same as ID2 !!";
      }
      else if(num1flag == false && num2flag == true)
      {
        document.getElementById('label').innerHTML = "Merge is fail !! No ID1 !!";
      }
      else if(num1flag == true && num2flag == false)
      {
        document.getElementById('label').innerHTML = "Merge is fail !! No ID2 !!";
      }
      else if(num1flag == false && num2flag == false)
      {
        document.getElementById('label').innerHTML = "Merge is fail !! No both ID !!";
      }
    }
  }  
  else if(document.getElementById("AbsolutePosition").style.display == "initial" || document.getElementById("AbsoluteSpeed").style.display ==  "initial")
  {
    
  }
}

function Position(){
  var xp = document.getElementById("X_series_position").value;
  var prop = Math.round(xp * 74.175824175824);
  document.getElementById("Pro_series_position").value = prop;
}

function PositionReverse(){
  var prop = document.getElementById("Pro_series_position").value;
  var xp = Math.round(prop / 74.175824175824);
  document.getElementById("X_series_position").value = xp;
}

function Speed(){
  var xs = document.getElementById("X_series_speed").value;
  var pros = Math.round(xs * 69.558772606601);
  document.getElementById("Pro_series_speed").value = pros;
}

function SpeedReverse(){
  var pros = document.getElementById("Pro_series_speed").value;
  var xs = Math.round(pros / 69.558772606601);
  document.getElementById("X_series_speed").value = xs;
}

// 全域變數記住狀態
const TOTAL_MOTORS = 30;   // 全身馬達數，總開關以「全部都開」為開啟狀態
var isTorqueOn = false; 

function TorqueSwitch() {
  var targetState = isTorqueOn ? 0 : 1;

  console.log("執行扭力切換: " + (targetState == 1 ? "ON" : "OFF"));
  var fullPackage = [83, 84, 246, 0, targetState, 78, 69];
  SendPackage.package = fullPackage;
  interface.publish(SendPackage);

  for (let i = 1; i <= TOTAL_MOTORS; i++) {
    motorTorqueStates[i] = targetState;
  }
  refreshTorqueUI();

  document.getElementById('label').innerHTML = targetState ? "Torque is ON" : "Torque is OFF";
}

// =======   馬達刻度   ========
/**
 * 這是用來更新馬達數值的核心函式
 * @param {Array} motorValues - 包含所有馬達數值的陣列 (例如: [512, 511, 2048...]) 
 * @param {string} partName - 部位名稱 (如 'left_hand')
 * @param {string} buttonID - HTML 按鈕的 ID
 * 假設 陣列 index 0 對應 ID 1, index 1 對應 ID 2... 以此類推
 */

// 機器人設定檔 (已補齊 1~29)
const robotConfig = {
    'left_hand':  [ { id: 1 }, { id: 2 }, { id: 3 }, { id: 4 }, { id: 5 }, { id: 6 }, { id: 7 }],
    'right_hand': [ { id: 8 }, { id: 9 }, { id: 10 }, { id: 11 }, { id: 12 }, { id: 13 }, { id: 14 } ],
    'waist':      [ { id: 15 }],
    'left_leg':   [ { id: 16 }, { id: 17 }, { id: 18 }, { id: 19 }, { id: 20 }, { id: 21 } ],
    'right_leg':  [ { id: 22 }, { id: 23 }, { id: 24 }, { id: 25 }, { id: 26 }, { id: 27 } ],
    // 頭部：driver_node 的 head_map 把邏輯 1/2/3 對應到實體 28/29/30
    'head':       [ { id: 28 }, { id: 29 } ,{ id: 30 } ],
};

// 紀錄目前哪些部位正在被「觀察/調整」
let activeParts = {
    'left_hand': false,
    'right_hand': false,
    'waist': false,
    'left_leg': false,
    'right_leg': false,
    'head': false,
};
// function Update_Motor_Input_Values(motorValues) {
//     // 遍歷所有部位
//     for (const partName in robotConfig) {
//         // 【關鍵判斷】只有被按下按鈕的部位 (activeParts 為 true) 才進行讀取顯示
//         if (activeParts[partName]) {
//             const motorsInPart = robotConfig[partName];
            
//             motorsInPart.forEach(motor => {
//                 // 假設 motorValues 陣列 index 0 對應 ID 1，以此類推
//                 let currentVal = motorValues[motor.id - 1]; 
//                 let inputField = document.getElementById('motor_val_' + motor.id);

//                 if (inputField) {
//                     // 如果使用者正在裡面打字，暫停自動更新以免干擾
//                     if (document.activeElement !== inputField) {
//                         inputField.value = currentVal;
//                     }
//                 }
//             });
//         }
//     }
// }
function Update_Motor_Input_Values(motorValues) {
    // 遍歷所有部位
    for (const partName in robotConfig) {
        
        // 移除 if (activeParts[partName]) 的限制，讓所有部位隨時更新
        const motorsInPart = robotConfig[partName];
        
        motorsInPart.forEach(motor => {
            // 假設 motorValues 陣列 index 0 對應 ID 1，以此類推
            let currentVal = motorValues[motor.id - 1]; 
            if (currentVal === undefined) return; // 這筆訊息沒有這顆，保留原值
            let inputField = document.getElementById('motor_val_' + motor.id);

            if (inputField) {
                // 如果使用者正在裡面打字，暫停自動更新以免干擾
                if (document.activeElement !== inputField) {
                    inputField.value = currentVal;
                }
            }
        });
        
    }
}

// =================================================================
// 輔助函式：整組開關扭力
// =================================================================
// =================================================================
// 扭力狀態的單一真實來源
//
// motorTorqueStates（InterfaceTable.js 宣告）是唯一的狀態，三個入口
// （部位按鈕 / 單顆開關 / 全身開關）都只負責送指令並更新它，
// 畫面一律由 refreshTorqueUI() 統一重繪，避免各自塗色而互相不同步。
// =================================================================
function refreshTorqueUI() {
    // 步驟一：人形圖上的單顆按鈕
    for (let id = 1; id <= TOTAL_MOTORS; id++) {
        const btn = document.getElementById(`pos-btn-${id}`);
        if (!btn) continue;
        btn.classList.toggle('torque-on', motorTorqueStates[id] === 1);
    }

    // 步驟二：部位按鈕，該部位「全部洩力」才算處於調整中
    for (const partName in robotConfig) {
        const motors = robotConfig[partName];
        const allOff = motors.every(m => motorTorqueStates[m.id] !== 1);
        activeParts[partName] = allOff;
        const btn = document.getElementById('btn_' + partName);
        if (btn) btn.classList.toggle('button_active', allOff);
    }

    // 步驟三：全身按鈕。只有「全部都開著」才算開啟狀態 ——
    // 只要有任何一顆沒開，按鈕就是關閉樣式，按下去會把全身都打開。
    let onCount = 0;
    for (let id = 1; id <= TOTAL_MOTORS; id++) {
        if (motorTorqueStates[id] === 1) onCount++;
    }
    isTorqueOn = (onCount === TOTAL_MOTORS);
    const allBtn = document.getElementById('TorqueButton');
    if (allBtn) {
        // 文字仍細分三種，才看得出目前實際開了幾顆
        if (onCount === 0)                 allBtn.innerText = '扭力已關閉';
        else if (onCount === TOTAL_MOTORS) allBtn.innerText = '扭力已開啟';
        else                               allBtn.innerText = '扭力部分開啟 (' + onCount + '/' + TOTAL_MOTORS + ')';
        allBtn.classList.toggle('active', isTorqueOn);
    }

    // 步驟四：單顆 ON/OFF 的高亮，跟著目前輸入的 ID 走
    const idInput = document.getElementById('single_motor_id');
    const btnOn = document.getElementById('btn_single_on');
    const btnOff = document.getElementById('btn_single_off');
    if (idInput && btnOn && btnOff) {
        const cur = motorTorqueStates[parseInt(idInput.value)] === 1;
        btnOn.classList.toggle('active', cur);
        btnOff.classList.toggle('active', !cur);
    }
}

function Torque_Choose(partName, buttonID) {
    const motors = robotConfig[partName];
    if (!motors) return;

    // 目前「全部洩力」就代表在調整中，再按一次要鎖回去；否則就是要洩力
    const allOff = motors.every(m => motorTorqueStates[m.id] !== 1);
    const targetState = allOff ? 1 : 0;

    SetGroupTorque(partName, targetState);
    motors.forEach(m => { motorTorqueStates[m.id] = targetState; });
    refreshTorqueUI();
}

// =================================================================
// 輔助函式：單顆開關扭力
// =================================================================
function SetSingleTorque(state) {
    const inputVal = document.getElementById('single_motor_id').value;
    const motorID = parseInt(inputVal);

    if (isNaN(motorID) || motorID < 1 || motorID > 30) {
        alert("請輸入有效的馬達 ID (1 ~ 30)！");
        return;
    }

    var dataPackage = [83, 84, 246, motorID, state, 78, 69];
    if (typeof SendPackage !== 'undefined' && typeof interface !== 'undefined') {
        SendPackage.sectorname = String(motorID);
        SendPackage.package = dataPackage;
        interface.publish(SendPackage);

        const stateText = state === 1 ? "ON (鎖死)" : "OFF (洩力)";
        document.getElementById('label').innerHTML = `Motor ID: ${motorID} Torque is ${stateText}`;

        motorTorqueStates[motorID] = state;
        refreshTorqueUI();
    }
}
function SetGroupTorque(partName, state) {
    const motors = robotConfig[partName];
    if (!motors) return;

    console.log(`[發送指令] 部位: ${partName}, 狀態: ${state === 0 ? '洩力(0)' : '鎖死(1)'}`);

    motors.forEach(motor => {
        // 封包格式: [83, 84, 246, ID, State, 78, 69]
        var dataPackage = [83, 84, 246, motor.id, state, 78, 69];
        
        // 【核心檢查】確保 SendPackage 和 interface 存在
        if (typeof SendPackage !== 'undefined' && typeof interface !== 'undefined') {
            
            // 為了防止全域變數覆寫，建議這裡要確保 publish 是立即執行的
            SendPackage.sectorname = String(motor.id);
            SendPackage.package = dataPackage;
            
            console.log(`-> 傳送給馬達 ID: ${motor.id}`, dataPackage);
            interface.publish(SendPackage);
            
        } else {
            console.error("錯誤：找不到 SendPackage 或 interface，指令無法送出！");
        }
    });
}