// 自動偵測目前是用哪個位址連上這個網頁，藉此決定要連哪個 rosbridge：
// - 透過 http://172.17.121.10:8080/... 開啟 → 自動連 172.17.121.10 (有線網路)
// - 透過 http://10.10.10.10:8080/...   開啟 → 自動連 10.10.10.10   (熱點)
// - 用 sftp 抓下來直接雙擊開啟 (file://)    → window.location.hostname 會是空字串，
//   fallback 回原本寫死的有線 IP，桌面端原本的操作方式不受影響
// 自動偵測目前是用哪個位址連上這個網頁，藉此決定預設要連哪個 rosbridge：
// - 透過 http://172.17.121.10:8080/... 開啟 → 自動連 172.17.121.10 (有線網路)
// - 透過 http://192.168.1.10:8080/...  開啟 → 自動連 192.168.1.10  (IClab_NETGEAR WiFi)
// - 透過 http://10.10.10.10:8080/...   開啟 → 自動連 10.10.10.10   (熱點)
// - 用 sftp 抓下來直接雙擊開啟 (file://)    → window.location.hostname 會是空字串，
//   fallback 回原本寫死的有線 IP，桌面端原本的操作方式不受影響
//
// 同時把所有已知網路的位址都列進下拉選單，方便手動切換
// (例如手機連著熱點，但想暫時切去連有線那台機器人測試)
// 用 SFTP (GVFS) 掛載開檔案時，網址會長這樣：
// file:///run/user/1000/gvfs/sftp:host=10.10.10.10,user=iclab/...
// hostname 抓不到值，但路徑裡藏著實際連線的 host，直接解析出來用。
function detectSftpHost() {
    var path = decodeURIComponent(window.location.pathname || '');
    var m = path.match(/sftp:host=([^,/]+)/);
    return m ? m[1] : null;
}

window.ROBOT_CONFIG = {
    ip: window.location.hostname || detectSftpHost() || '172.17.121.10'
};

// 之後新增/修改已知網路位址，只要改這個清單即可，不用動下面的邏輯
window.ROBOT_KNOWN_ADDRESSES = [
    { label: '有線網路',        ip: '172.17.121.10' },
    { label: 'IClab_NETGEAR WiFi', ip: '192.168.1.10' },
    { label: '機器人熱點',      ip: '10.10.10.10' },
];

document.addEventListener('DOMContentLoaded', function () {
    var sel = document.getElementById('addressSelect');
    if (!sel) return;

    // 先把目前自動偵測到的位址插在最前面，預設選中它
    var detectedOpt = document.createElement('option');
    detectedOpt.value = window.ROBOT_CONFIG.ip;
    detectedOpt.text  = window.ROBOT_CONFIG.ip + ' (目前)';
    sel.insertBefore(detectedOpt, sel.firstChild);

    // 再把其他已知位址列進去，跳過跟目前偵測到的重複的那一個
    window.ROBOT_KNOWN_ADDRESSES.forEach(function (entry) {
        if (entry.ip === window.ROBOT_CONFIG.ip) return;
        var opt = document.createElement('option');
        opt.value = entry.ip;
        opt.text  = entry.label + ' (' + entry.ip + ')';
        sel.appendChild(opt);
    });

    sel.value = window.ROBOT_CONFIG.ip;
});
