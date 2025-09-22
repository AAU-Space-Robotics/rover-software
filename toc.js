// Populate the sidebar
//
// This is a script, and not included directly in the page, to control the total size of the book.
// The TOC contains an entry for each page, so if each page includes a copy of the TOC,
// the total size of the page becomes O(n**2).
class MDBookSidebarScrollbox extends HTMLElement {
    constructor() {
        super();
    }
    connectedCallback() {
        this.innerHTML = '<ol class="chapter"><li class="chapter-item expanded "><a href="introduction.html"><strong aria-hidden="true">1.</strong> Overview</a></li><li class="chapter-item expanded affix "><li class="part-title">Getting Started</li><li class="chapter-item expanded "><a href="getting_started/launching.html"><strong aria-hidden="true">2.</strong> Launching</a></li><li class="chapter-item expanded "><a href="getting_started/development-setup.html"><strong aria-hidden="true">3.</strong> Development Setup</a></li><li class="chapter-item expanded "><a href="getting_started/development-vs-production.html"><strong aria-hidden="true">4.</strong> Development vs Production</a></li><li class="chapter-item expanded affix "><li class="part-title">ROS2 Stack</li><li class="chapter-item expanded "><a href="ros2-stack/topics-and-messages.html"><strong aria-hidden="true">5.</strong> Topics and Messages</a></li><li class="chapter-item expanded "><div><strong aria-hidden="true">6.</strong> Navigation</div></li><li><ol class="section"><li class="chapter-item expanded "><div><strong aria-hidden="true">6.1.</strong> Perception</div></li><li class="chapter-item expanded "><div><strong aria-hidden="true">6.2.</strong> Planning</div></li><li class="chapter-item expanded "><div><strong aria-hidden="true">6.3.</strong> Control</div></li><li class="chapter-item expanded "><div><strong aria-hidden="true">6.4.</strong> Localization</div></li><li class="chapter-item expanded "><div><strong aria-hidden="true">6.5.</strong> Mapping</div></li></ol></li><li class="chapter-item expanded "><div><strong aria-hidden="true">7.</strong> Manipulation</div></li><li class="chapter-item expanded "><div><strong aria-hidden="true">8.</strong> Simulation</div></li><li><ol class="section"><li class="chapter-item expanded "><div><strong aria-hidden="true">8.1.</strong> Isaac Sim</div></li><li class="chapter-item expanded "><div><strong aria-hidden="true">8.2.</strong> Gazebo</div></li></ol></li><li class="chapter-item expanded "><li class="part-title">Hardware Specifications</li><li class="chapter-item expanded "><div><strong aria-hidden="true">9.</strong> Robot Specifications</div></li><li><ol class="section"><li class="chapter-item expanded "><div><strong aria-hidden="true">9.1.</strong> Sensor Specifications</div></li><li class="chapter-item expanded "><div><strong aria-hidden="true">9.2.</strong> Actuator Specifications</div></li><li class="chapter-item expanded "><div><strong aria-hidden="true">9.3.</strong> Compute Specifications</div></li><li class="chapter-item expanded "><div><strong aria-hidden="true">9.4.</strong> Power Specifications</div></li></ol></li><li class="chapter-item expanded "><div><strong aria-hidden="true">10.</strong> Kinematics</div></li><li class="chapter-item expanded affix "><li class="part-title">Contributing</li><li class="chapter-item expanded "><a href="contributing/overview.html"><strong aria-hidden="true">11.</strong> Overview</a></li><li class="chapter-item expanded "><a href="contributing/branching.html"><strong aria-hidden="true">12.</strong> Branching Strategy</a></li><li class="chapter-item expanded "><a href="contributing/commits.html"><strong aria-hidden="true">13.</strong> Commit Guidelines</a></li><li class="chapter-item expanded "><a href="contributing/pull-requests.html"><strong aria-hidden="true">14.</strong> Pull Requests</a></li><li class="chapter-item expanded "><a href="contributing/code-style.html"><strong aria-hidden="true">15.</strong> Code Style</a></li><li class="chapter-item expanded affix "><li class="part-title">Advanced</li><li class="chapter-item expanded "><a href="advanced/installation.html"><strong aria-hidden="true">16.</strong> Installation (OS Reinstall)</a></li></ol>';
        // Set the current, active page, and reveal it if it's hidden
        let current_page = document.location.href.toString().split("#")[0].split("?")[0];
        if (current_page.endsWith("/")) {
            current_page += "index.html";
        }
        var links = Array.prototype.slice.call(this.querySelectorAll("a"));
        var l = links.length;
        for (var i = 0; i < l; ++i) {
            var link = links[i];
            var href = link.getAttribute("href");
            if (href && !href.startsWith("#") && !/^(?:[a-z+]+:)?\/\//.test(href)) {
                link.href = path_to_root + href;
            }
            // The "index" page is supposed to alias the first chapter in the book.
            if (link.href === current_page || (i === 0 && path_to_root === "" && current_page.endsWith("/index.html"))) {
                link.classList.add("active");
                var parent = link.parentElement;
                if (parent && parent.classList.contains("chapter-item")) {
                    parent.classList.add("expanded");
                }
                while (parent) {
                    if (parent.tagName === "LI" && parent.previousElementSibling) {
                        if (parent.previousElementSibling.classList.contains("chapter-item")) {
                            parent.previousElementSibling.classList.add("expanded");
                        }
                    }
                    parent = parent.parentElement;
                }
            }
        }
        // Track and set sidebar scroll position
        this.addEventListener('click', function(e) {
            if (e.target.tagName === 'A') {
                sessionStorage.setItem('sidebar-scroll', this.scrollTop);
            }
        }, { passive: true });
        var sidebarScrollTop = sessionStorage.getItem('sidebar-scroll');
        sessionStorage.removeItem('sidebar-scroll');
        if (sidebarScrollTop) {
            // preserve sidebar scroll position when navigating via links within sidebar
            this.scrollTop = sidebarScrollTop;
        } else {
            // scroll sidebar to current active section when navigating via "next/previous chapter" buttons
            var activeSection = document.querySelector('#sidebar .active');
            if (activeSection) {
                activeSection.scrollIntoView({ block: 'center' });
            }
        }
        // Toggle buttons
        var sidebarAnchorToggles = document.querySelectorAll('#sidebar a.toggle');
        function toggleSection(ev) {
            ev.currentTarget.parentElement.classList.toggle('expanded');
        }
        Array.from(sidebarAnchorToggles).forEach(function (el) {
            el.addEventListener('click', toggleSection);
        });
    }
}
window.customElements.define("mdbook-sidebar-scrollbox", MDBookSidebarScrollbox);
