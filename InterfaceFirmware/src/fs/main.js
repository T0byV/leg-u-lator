"use strict"

let banners = document.getElementsByClassName("banner")
for (let banner of banners) {
    if (banner.textContent.trim() === "")
        banner.style.display = "none";

}
