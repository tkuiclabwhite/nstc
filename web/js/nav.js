// 全站導覽列：要加頁、改順序、改名稱，只改這個檔案，所有頁面一起生效。
//
// 用法：頁面的 <head> 裡載入這支，body 裡把原本那一整塊手寫的 <ul>...</ul>
// 換成一行空的容器即可：
//
//     <script src="./js/nav.js"></script>     <!-- head -->
//     <ul id="mainnav"></ul>                  <!-- body -->
//
// 目前頁面會自動加底線，不用再像以前那樣每頁手寫 style="text-decoration:underline"。
// 如果目前頁面在某個下拉選單裡，會同時給該下拉標籤加底線、給項目加 .dm-active
// （沿用 css/style.css 既有的規則，不用動 CSS）。
//
// 手機版（MobileJoystick）的導覽列項目不一樣，用容器上的 data-nav 指定要用哪一組：
//
//     <ul id="mainnav" data-nav="mobile"></ul>

// ===========================================================================
// 導覽列內容 —— 要改的就是這裡
// ===========================================================================
window.ROBOT_NAV_MENUS = {

    // 桌面版，12 個頁面共用。
    // 注意 css/style.css 裡 li 是 float:right，所以「排在前面的會靠右」。
    // 這個順序呈現出來（左→右）是：logo ... Tools, Hand, ImageProcess, Motion, Walking
    main: [
        { logo: true },
        { label: 'Motion', href: 'MotionControlInterface.html' },
        { label: 'ImageProcess', href: 'ImageProcessInterface.html' },
        {
            label: 'Hand', dropdown: [
                { label: 'HandControl', href: 'HandControlInterface.html' },
                { label: 'HandTouch', href: 'HandTouchInterface.html' },
                { label: 'HandMotion', href: 'HandMotionInterface.html' },
            ]
        },
    ],

    // 手機版：nstc 沒有手機頁，留空
    mobile: [],
};

// logo 圖片與連結，兩組選單共用
window.ROBOT_NAV_LOGO = {
    href: 'ImageProcessInterface.html',
    img: './resources/TKU_ICLab/iclablogo-02.png',
    width: '226px',
};

// ===========================================================================
// 以下是產生 DOM 的部分，平常不用動
// ===========================================================================
(function () {
    // 目前是哪一頁。用 sftp 掛載以 file:// 開啟時路徑會很長，
    // 所以只比對最後一段檔名，不比對完整路徑。
    function currentPage() {
        var path = decodeURIComponent(window.location.pathname || '');
        var name = path.split('/').pop();
        return (name || 'index.html').toLowerCase();
    }

    function samePage(href, cur) {
        return String(href).split('/').pop().toLowerCase() === cur;
    }

    function makeLogo() {
        var cfg = window.ROBOT_NAV_LOGO;
        var li = document.createElement('li');
        li.className = 'logo';
        li.style.width = cfg.width;
        var a = document.createElement('a');
        a.target = '_blank';
        a.href = cfg.href;
        a.className = 'logo';
        var img = document.createElement('img');
        img.id = 'logoimg';
        img.src = cfg.img;
        a.appendChild(img);
        li.appendChild(a);
        return li;
    }

    function makeLink(item) {
        var a = document.createElement('a');
        a.target = '_blank';
        a.className = 'navbarright';
        a.href = item.href;
        a.textContent = item.label;
        var li = document.createElement('li');
        li.appendChild(a);
        return { li: li, a: a };
    }

    function makeDropdown(item, cur) {
        var li = document.createElement('li');
        li.className = 'dropdown';

        var label = document.createElement('a');
        label.className = 'navbarright';
        label.href = '#';
        label.textContent = item.label + ' ▾';   // ▾
        li.appendChild(label);

        var menu = document.createElement('ul');
        menu.className = 'dropdown-menu';
        var hit = false;
        item.dropdown.forEach(function (sub) {
            var subLi = document.createElement('li');
            var subA = document.createElement('a');
            subA.target = '_blank';
            subA.href = sub.href;
            subA.textContent = sub.label;
            if (samePage(sub.href, cur)) {
                // .dm-active 的樣式已經在 css/style.css 裡
                subA.className = 'dm-active';
                hit = true;
            }
            subLi.appendChild(subA);
            menu.appendChild(subLi);
        });
        li.appendChild(menu);

        // 沿用原本各頁的慣例：目前頁在下拉裡時，下拉標籤本身也要加底線
        if (hit) label.style.textDecoration = 'underline';
        return li;
    }

    function render() {
        var ul = document.getElementById('mainnav');
        if (!ul) return;   // 這頁沒有導覽列（例如 API.html）

        var which = ul.getAttribute('data-nav') || 'main';
        var items = window.ROBOT_NAV_MENUS[which];
        if (!items) {
            console.warn('nav.js: 找不到選單 "' + which + '"');
            return;
        }

        var cur = currentPage();
        ul.innerHTML = '';
        items.forEach(function (item) {
            if (item.logo) {
                ul.appendChild(makeLogo());
            } else if (item.dropdown) {
                ul.appendChild(makeDropdown(item, cur));
            } else {
                var made = makeLink(item);
                if (samePage(item.href, cur)) {
                    made.a.style.textDecoration = 'underline';
                }
                ul.appendChild(made.li);
            }
        });
    }

    // 這支通常放在 <head>，但萬一被改成 defer 或動態載入，DOM 可能已經好了
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', render);
    } else {
        render();
    }
})();
