


<!DOCTYPE html>
<html lang="en" class=" is-copy-enabled">
  <head prefix="og: http://ogp.me/ns# fb: http://ogp.me/ns/fb# object: http://ogp.me/ns/object# article: http://ogp.me/ns/article# profile: http://ogp.me/ns/profile#">
    <meta charset='utf-8'>
    <meta http-equiv="X-UA-Compatible" content="IE=edge">
    <meta http-equiv="Content-Language" content="en">
    <meta name="viewport" content="width=1020">
    
    
    <title>dtalite_beta_test/EmissionsOutput.cpp at master · xzhou99/dtalite_beta_test</title>
    <link rel="search" type="application/opensearchdescription+xml" href="/opensearch.xml" title="GitHub">
    <link rel="fluid-icon" href="https://github.com/fluidicon.png" title="GitHub">
    <link rel="apple-touch-icon" sizes="57x57" href="/apple-touch-icon-114.png">
    <link rel="apple-touch-icon" sizes="114x114" href="/apple-touch-icon-114.png">
    <link rel="apple-touch-icon" sizes="72x72" href="/apple-touch-icon-144.png">
    <link rel="apple-touch-icon" sizes="144x144" href="/apple-touch-icon-144.png">
    <meta property="fb:app_id" content="1401488693436528">

      <meta content="@github" name="twitter:site" /><meta content="summary" name="twitter:card" /><meta content="xzhou99/dtalite_beta_test" name="twitter:title" /><meta content="Contribute to dtalite_beta_test development by creating an account on GitHub." name="twitter:description" /><meta content="https://avatars0.githubusercontent.com/u/3403345?v=3&amp;s=400" name="twitter:image:src" />
      <meta content="GitHub" property="og:site_name" /><meta content="object" property="og:type" /><meta content="https://avatars0.githubusercontent.com/u/3403345?v=3&amp;s=400" property="og:image" /><meta content="xzhou99/dtalite_beta_test" property="og:title" /><meta content="https://github.com/xzhou99/dtalite_beta_test" property="og:url" /><meta content="Contribute to dtalite_beta_test development by creating an account on GitHub." property="og:description" />
      <meta name="browser-stats-url" content="https://api.github.com/_private/browser/stats">
    <meta name="browser-errors-url" content="https://api.github.com/_private/browser/errors">
    <link rel="assets" href="https://assets-cdn.github.com/">
    <link rel="web-socket" href="wss://live.github.com/_sockets/NzYzMzY4NTowODVmZTQwOWRiMmIyNjI3ZDU0ODQyZTk4ZWUxMDU0OTpkZjNhZmQ2OWYyMmY5OTJkMzk5NjA5MWNmNmE4ZWQ3MTg4ZWI0N2QzNmRkZDFjM2NlMDAyODcyZWQyY2ZmYWFk--fe9af9c10d24836725a0190b20e70ea7cb83aa4d">
    <meta name="pjax-timeout" content="1000">
    <link rel="sudo-modal" href="/sessions/sudo_modal">

    <meta name="msapplication-TileImage" content="/windows-tile.png">
    <meta name="msapplication-TileColor" content="#ffffff">
    <meta name="selected-link" value="repo_source" data-pjax-transient>

    <meta name="google-site-verification" content="KT5gs8h0wvaagLKAVWq8bbeNwnZZK1r1XQysX3xurLU">
    <meta name="google-analytics" content="UA-3769691-2">

<meta content="collector.githubapp.com" name="octolytics-host" /><meta content="collector-cdn.github.com" name="octolytics-script-host" /><meta content="github" name="octolytics-app-id" /><meta content="9807E009:362A:114085C2:560EF467" name="octolytics-dimension-request_id" /><meta content="7633685" name="octolytics-actor-id" /><meta content="stanvir" name="octolytics-actor-login" /><meta content="1e2a78285eea43688a6049b49712d1702d99596907261ed4480944238631b012" name="octolytics-actor-hash" />

<meta content="Rails, view, blob#show" data-pjax-transient="true" name="analytics-event" />


  <meta class="js-ga-set" name="dimension1" content="Logged In">
    <meta class="js-ga-set" name="dimension4" content="Current repo nav">




    <meta name="is-dotcom" content="true">
        <meta name="hostname" content="github.com">
    <meta name="user-login" content="stanvir">

      <link rel="mask-icon" href="https://assets-cdn.github.com/pinned-octocat.svg" color="#4078c0">
      <link rel="icon" type="image/x-icon" href="https://assets-cdn.github.com/favicon.ico">

      <!-- </textarea> --><!-- '"` --><meta content="authenticity_token" name="csrf-param" />
<meta content="HVMQWVm3n0MAfM0sPPqqO+Gd4UvD3Svhpul8S3o0bGL68BfgNO5QkaRQicnEkUE1pA5Oh9MTDhJYvdhLE6UGgw==" name="csrf-token" />
    <meta content="a7483060e72f5a6ca485393f7ddb2f2d156361be" name="form-nonce" />

    <link crossorigin="anonymous" href="https://assets-cdn.github.com/assets/github-9b1f8fe15d1bcf0f1e67cd7bab04441c73da6150fa7271bfe6c01632e4a4096d.css" integrity="sha256-mx+P4V0bzw8eZ817qwREHHPaYVD6cnG/5sAWMuSkCW0=" media="all" rel="stylesheet" />
    <link crossorigin="anonymous" href="https://assets-cdn.github.com/assets/github2-8cb29b7d57abf31a52e83ed83d302775783540e2a0877c0e24d6c21718464e1d.css" integrity="sha256-jLKbfVer8xpS6D7YPTAndXg1QOKgh3wOJNbCFxhGTh0=" media="all" rel="stylesheet" />
    
    
    


    <meta http-equiv="x-pjax-version" content="8cbe519d90e172a29f6fbe0ecca26be4">

      
  <meta name="description" content="Contribute to dtalite_beta_test development by creating an account on GitHub.">
  <meta name="go-import" content="github.com/xzhou99/dtalite_beta_test git https://github.com/xzhou99/dtalite_beta_test.git">

  <meta content="3403345" name="octolytics-dimension-user_id" /><meta content="xzhou99" name="octolytics-dimension-user_login" /><meta content="42977025" name="octolytics-dimension-repository_id" /><meta content="xzhou99/dtalite_beta_test" name="octolytics-dimension-repository_nwo" /><meta content="true" name="octolytics-dimension-repository_public" /><meta content="false" name="octolytics-dimension-repository_is_fork" /><meta content="42977025" name="octolytics-dimension-repository_network_root_id" /><meta content="xzhou99/dtalite_beta_test" name="octolytics-dimension-repository_network_root_nwo" />
  <link href="https://github.com/xzhou99/dtalite_beta_test/commits/master.atom" rel="alternate" title="Recent Commits to dtalite_beta_test:master" type="application/atom+xml">

  </head>


  <body class="logged_in   env-production windows vis-public page-blob">
    <a href="#start-of-content" tabindex="1" class="accessibility-aid js-skip-to-content">Skip to content</a>

    
    
    



      <div class="header header-logged-in true" role="banner">
  <div class="container clearfix">

    <a class="header-logo-invertocat" href="https://github.com/" data-hotkey="g d" aria-label="Homepage" data-ga-click="Header, go to dashboard, icon:logo">
  <span class="mega-octicon octicon-mark-github"></span>
</a>


      <div class="site-search repo-scope js-site-search" role="search">
          <!-- </textarea> --><!-- '"` --><form accept-charset="UTF-8" action="/xzhou99/dtalite_beta_test/search" class="js-site-search-form" data-global-search-url="/search" data-repo-search-url="/xzhou99/dtalite_beta_test/search" method="get"><div style="margin:0;padding:0;display:inline"><input name="utf8" type="hidden" value="&#x2713;" /></div>
  <label class="js-chromeless-input-container form-control">
    <div class="scope-badge">This repository</div>
    <input type="text"
      class="js-site-search-focus js-site-search-field is-clearable chromeless-input"
      data-hotkey="s"
      name="q"
      placeholder="Search"
      aria-label="Search this repository"
      data-global-scope-placeholder="Search GitHub"
      data-repo-scope-placeholder="Search"
      tabindex="1"
      autocapitalize="off">
  </label>
</form>
      </div>

      <ul class="header-nav left" role="navigation">
        <li class="header-nav-item">
          <a href="/pulls" class="js-selected-navigation-item header-nav-link" data-ga-click="Header, click, Nav menu - item:pulls context:user" data-hotkey="g p" data-selected-links="/pulls /pulls/assigned /pulls/mentioned /pulls">
            Pull requests
</a>        </li>
        <li class="header-nav-item">
          <a href="/issues" class="js-selected-navigation-item header-nav-link" data-ga-click="Header, click, Nav menu - item:issues context:user" data-hotkey="g i" data-selected-links="/issues /issues/assigned /issues/mentioned /issues">
            Issues
</a>        </li>
          <li class="header-nav-item">
            <a class="header-nav-link" href="https://gist.github.com/" data-ga-click="Header, go to gist, text:gist">Gist</a>
          </li>
      </ul>

    
<ul class="header-nav user-nav right" id="user-links">
  <li class="header-nav-item">
      <span class="js-socket-channel js-updatable-content"
        data-channel="notification-changed:stanvir"
        data-url="/notifications/header">
      <a href="/notifications" aria-label="You have no unread notifications" class="header-nav-link notification-indicator tooltipped tooltipped-s" data-ga-click="Header, go to notifications, icon:read" data-hotkey="g n">
          <span class="mail-status all-read"></span>
          <span class="octicon octicon-bell"></span>
</a>  </span>

  </li>

  <li class="header-nav-item dropdown js-menu-container">
    <a class="header-nav-link tooltipped tooltipped-s js-menu-target" href="/new"
       aria-label="Create new…"
       data-ga-click="Header, create new, icon:add">
      <span class="octicon octicon-plus left"></span>
      <span class="dropdown-caret"></span>
    </a>

    <div class="dropdown-menu-content js-menu-content">
      <ul class="dropdown-menu dropdown-menu-sw">
        
<a class="dropdown-item" href="/new" data-ga-click="Header, create new repository">
  New repository
</a>


  <a class="dropdown-item" href="/organizations/new" data-ga-click="Header, create new organization">
    New organization
  </a>



  <div class="dropdown-divider"></div>
  <div class="dropdown-header">
    <span title="xzhou99/dtalite_beta_test">This repository</span>
  </div>
    <a class="dropdown-item" href="/xzhou99/dtalite_beta_test/issues/new" data-ga-click="Header, create new issue">
      New issue
    </a>

      </ul>
    </div>
  </li>

  <li class="header-nav-item dropdown js-menu-container">
    <a class="header-nav-link name tooltipped tooltipped-s js-menu-target" href="/stanvir"
       aria-label="View profile and more"
       data-ga-click="Header, show menu, icon:avatar">
      <img alt="@stanvir" class="avatar" height="20" src="https://avatars0.githubusercontent.com/u/7633685?v=3&amp;s=40" width="20" />
      <span class="dropdown-caret"></span>
    </a>

    <div class="dropdown-menu-content js-menu-content">
      <div class="dropdown-menu  dropdown-menu-sw">
        <div class=" dropdown-header header-nav-current-user css-truncate">
            Signed in as <strong class="css-truncate-target">stanvir</strong>

        </div>


        <div class="dropdown-divider"></div>

          <a class="dropdown-item" href="/stanvir" data-ga-click="Header, go to profile, text:your profile">
            Your profile
          </a>
        <a class="dropdown-item" href="/stars" data-ga-click="Header, go to starred repos, text:your stars">
          Your stars
        </a>
        <a class="dropdown-item" href="/explore" data-ga-click="Header, go to explore, text:explore">
          Explore
        </a>
          <a class="dropdown-item" href="/integrations" data-ga-click="Header, go to integrations, text:integrations">
            Integrations
          </a>
        <a class="dropdown-item" href="https://help.github.com" data-ga-click="Header, go to help, text:help">
          Help
        </a>

          <div class="dropdown-divider"></div>

          <a class="dropdown-item" href="/settings/profile" data-ga-click="Header, go to settings, icon:settings">
            Settings
          </a>

          <!-- </textarea> --><!-- '"` --><form accept-charset="UTF-8" action="/logout" class="logout-form" data-form-nonce="a7483060e72f5a6ca485393f7ddb2f2d156361be" method="post"><div style="margin:0;padding:0;display:inline"><input name="utf8" type="hidden" value="&#x2713;" /><input name="authenticity_token" type="hidden" value="cW4PmsMCzP4nMU3Kj9+VGGuPAdOFzPM/Ham6BC0wJQ0m8XSUYWoHs4cilO06IEqSefgLigEBRilLABuEgZShRg==" /></div>
            <button class="dropdown-item dropdown-signout" data-ga-click="Header, sign out, icon:logout">
              Sign out
            </button>
</form>
      </div>
    </div>
  </li>
</ul>


    
  </div>
</div>

      

      


    <div id="start-of-content" class="accessibility-aid"></div>

    <div id="js-flash-container">
</div>


    <div role="main" class="main-content">
        <div itemscope itemtype="http://schema.org/WebPage">
    <div class="pagehead repohead instapaper_ignore readability-menu">

      <div class="container">

        <div class="clearfix">
          
<ul class="pagehead-actions">

  <li>
      <!-- </textarea> --><!-- '"` --><form accept-charset="UTF-8" action="/notifications/subscribe" class="js-social-container" data-autosubmit="true" data-form-nonce="a7483060e72f5a6ca485393f7ddb2f2d156361be" data-remote="true" method="post"><div style="margin:0;padding:0;display:inline"><input name="utf8" type="hidden" value="&#x2713;" /><input name="authenticity_token" type="hidden" value="4cMZEb4/zRM3C2b2U8wV/ZPyNMBZUyvmY9wMYfwwz2cJGCttJtmS2g+UGt0dJZc4kYtjPCRg3jyXeInL09AMlQ==" /></div>    <input id="repository_id" name="repository_id" type="hidden" value="42977025" />

      <div class="select-menu js-menu-container js-select-menu">
        <a href="/xzhou99/dtalite_beta_test/subscription"
          class="btn btn-sm btn-with-count select-menu-button js-menu-target" role="button" tabindex="0" aria-haspopup="true"
          data-ga-click="Repository, click Watch settings, action:blob#show">
          <span class="js-select-button">
            <span class="octicon octicon-eye"></span>
            Watch
          </span>
        </a>
        <a class="social-count js-social-count" href="/xzhou99/dtalite_beta_test/watchers">
          3
        </a>

        <div class="select-menu-modal-holder">
          <div class="select-menu-modal subscription-menu-modal js-menu-content" aria-hidden="true">
            <div class="select-menu-header">
              <span class="select-menu-title">Notifications</span>
              <span class="octicon octicon-x js-menu-close" role="button" aria-label="Close"></span>
            </div>

            <div class="select-menu-list js-navigation-container" role="menu">

              <div class="select-menu-item js-navigation-item selected" role="menuitem" tabindex="0">
                <span class="select-menu-item-icon octicon octicon-check"></span>
                <div class="select-menu-item-text">
                  <input checked="checked" id="do_included" name="do" type="radio" value="included" />
                  <span class="select-menu-item-heading">Not watching</span>
                  <span class="description">Be notified when participating or @mentioned.</span>
                  <span class="js-select-button-text hidden-select-button-text">
                    <span class="octicon octicon-eye"></span>
                    Watch
                  </span>
                </div>
              </div>

              <div class="select-menu-item js-navigation-item " role="menuitem" tabindex="0">
                <span class="select-menu-item-icon octicon octicon octicon-check"></span>
                <div class="select-menu-item-text">
                  <input id="do_subscribed" name="do" type="radio" value="subscribed" />
                  <span class="select-menu-item-heading">Watching</span>
                  <span class="description">Be notified of all conversations.</span>
                  <span class="js-select-button-text hidden-select-button-text">
                    <span class="octicon octicon-eye"></span>
                    Unwatch
                  </span>
                </div>
              </div>

              <div class="select-menu-item js-navigation-item " role="menuitem" tabindex="0">
                <span class="select-menu-item-icon octicon octicon-check"></span>
                <div class="select-menu-item-text">
                  <input id="do_ignore" name="do" type="radio" value="ignore" />
                  <span class="select-menu-item-heading">Ignoring</span>
                  <span class="description">Never be notified.</span>
                  <span class="js-select-button-text hidden-select-button-text">
                    <span class="octicon octicon-mute"></span>
                    Stop ignoring
                  </span>
                </div>
              </div>

            </div>

          </div>
        </div>
      </div>
</form>
  </li>

  <li>
    
  <div class="js-toggler-container js-social-container starring-container ">

    <!-- </textarea> --><!-- '"` --><form accept-charset="UTF-8" action="/xzhou99/dtalite_beta_test/unstar" class="js-toggler-form starred js-unstar-button" data-form-nonce="a7483060e72f5a6ca485393f7ddb2f2d156361be" data-remote="true" method="post"><div style="margin:0;padding:0;display:inline"><input name="utf8" type="hidden" value="&#x2713;" /><input name="authenticity_token" type="hidden" value="99CKhWQbvNBpBWDCMsUuMRW2ehA1U6hnjxalKxcolXErw4v1pa9g+FK9sHxPK/uAyJYkkwIbAfrkVvTRzFeGpw==" /></div>
      <button
        class="btn btn-sm btn-with-count js-toggler-target"
        aria-label="Unstar this repository" title="Unstar xzhou99/dtalite_beta_test"
        data-ga-click="Repository, click unstar button, action:blob#show; text:Unstar">
        <span class="octicon octicon-star"></span>
        Unstar
      </button>
        <a class="social-count js-social-count" href="/xzhou99/dtalite_beta_test/stargazers">
          0
        </a>
</form>
    <!-- </textarea> --><!-- '"` --><form accept-charset="UTF-8" action="/xzhou99/dtalite_beta_test/star" class="js-toggler-form unstarred js-star-button" data-form-nonce="a7483060e72f5a6ca485393f7ddb2f2d156361be" data-remote="true" method="post"><div style="margin:0;padding:0;display:inline"><input name="utf8" type="hidden" value="&#x2713;" /><input name="authenticity_token" type="hidden" value="YQe65WjPUzdcu7kqQX5EUConffv69ypl797IKZO9ktIf1gjpzw4sorQoDuLrRQXP43uUG2jwN7CL+D2Q39HDlw==" /></div>
      <button
        class="btn btn-sm btn-with-count js-toggler-target"
        aria-label="Star this repository" title="Star xzhou99/dtalite_beta_test"
        data-ga-click="Repository, click star button, action:blob#show; text:Star">
        <span class="octicon octicon-star"></span>
        Star
      </button>
        <a class="social-count js-social-count" href="/xzhou99/dtalite_beta_test/stargazers">
          0
        </a>
</form>  </div>

  </li>

  <li>
          <a href="#fork-destination-box" class="btn btn-sm btn-with-count"
              title="Fork your own copy of xzhou99/dtalite_beta_test to your account"
              aria-label="Fork your own copy of xzhou99/dtalite_beta_test to your account"
              rel="facebox"
              data-ga-click="Repository, show fork modal, action:blob#show; text:Fork">
            <span class="octicon octicon-repo-forked"></span>
            Fork
          </a>

          <div id="fork-destination-box" style="display: none;">
            <h2 class="facebox-header" data-facebox-id="facebox-header">Where should we fork this repository?</h2>
            <include-fragment src=""
                class="js-fork-select-fragment fork-select-fragment"
                data-url="/xzhou99/dtalite_beta_test/fork?fragment=1">
              <img alt="Loading" height="64" src="https://assets-cdn.github.com/images/spinners/octocat-spinner-128.gif" width="64" />
            </include-fragment>
          </div>

    <a href="/xzhou99/dtalite_beta_test/network" class="social-count">
      1
    </a>
  </li>
</ul>

          <h1 itemscope itemtype="http://data-vocabulary.org/Breadcrumb" class="entry-title public ">
  <span class="mega-octicon octicon-repo"></span>
  <span class="author"><a href="/xzhou99" class="url fn" itemprop="url" rel="author"><span itemprop="title">xzhou99</span></a></span><!--
--><span class="path-divider">/</span><!--
--><strong><a href="/xzhou99/dtalite_beta_test" data-pjax="#js-repo-pjax-container">dtalite_beta_test</a></strong>

  <span class="page-context-loader">
    <img alt="" height="16" src="https://assets-cdn.github.com/images/spinners/octocat-spinner-32.gif" width="16" />
  </span>

</h1>

        </div>
      </div>
    </div>

    <div class="container">
      <div class="repository-with-sidebar repo-container new-discussion-timeline ">
        <div class="repository-sidebar clearfix">
          
<nav class="sunken-menu repo-nav js-repo-nav js-sidenav-container-pjax js-octicon-loaders"
     role="navigation"
     data-pjax="#js-repo-pjax-container"
     data-issue-count-url="/xzhou99/dtalite_beta_test/issues/counts">
  <ul class="sunken-menu-group">
    <li class="tooltipped tooltipped-w" aria-label="Code">
      <a href="/xzhou99/dtalite_beta_test" aria-label="Code" aria-selected="true" class="js-selected-navigation-item selected sunken-menu-item" data-hotkey="g c" data-selected-links="repo_source repo_downloads repo_commits repo_releases repo_tags repo_branches /xzhou99/dtalite_beta_test">
        <span class="octicon octicon-code"></span> <span class="full-word">Code</span>
        <img alt="" class="mini-loader" height="16" src="https://assets-cdn.github.com/images/spinners/octocat-spinner-32.gif" width="16" />
</a>    </li>

      <li class="tooltipped tooltipped-w" aria-label="Issues">
        <a href="/xzhou99/dtalite_beta_test/issues" aria-label="Issues" class="js-selected-navigation-item sunken-menu-item" data-hotkey="g i" data-selected-links="repo_issues repo_labels repo_milestones /xzhou99/dtalite_beta_test/issues">
          <span class="octicon octicon-issue-opened"></span> <span class="full-word">Issues</span>
          <span class="js-issue-replace-counter"></span>
          <img alt="" class="mini-loader" height="16" src="https://assets-cdn.github.com/images/spinners/octocat-spinner-32.gif" width="16" />
</a>      </li>

    <li class="tooltipped tooltipped-w" aria-label="Pull requests">
      <a href="/xzhou99/dtalite_beta_test/pulls" aria-label="Pull requests" class="js-selected-navigation-item sunken-menu-item" data-hotkey="g p" data-selected-links="repo_pulls /xzhou99/dtalite_beta_test/pulls">
          <span class="octicon octicon-git-pull-request"></span> <span class="full-word">Pull requests</span>
          <span class="js-pull-replace-counter"></span>
          <img alt="" class="mini-loader" height="16" src="https://assets-cdn.github.com/images/spinners/octocat-spinner-32.gif" width="16" />
</a>    </li>

      <li class="tooltipped tooltipped-w" aria-label="Wiki">
        <a href="/xzhou99/dtalite_beta_test/wiki" aria-label="Wiki" class="js-selected-navigation-item sunken-menu-item" data-hotkey="g w" data-selected-links="repo_wiki /xzhou99/dtalite_beta_test/wiki">
          <span class="octicon octicon-book"></span> <span class="full-word">Wiki</span>
          <img alt="" class="mini-loader" height="16" src="https://assets-cdn.github.com/images/spinners/octocat-spinner-32.gif" width="16" />
</a>      </li>
  </ul>
  <div class="sunken-menu-separator"></div>
  <ul class="sunken-menu-group">

    <li class="tooltipped tooltipped-w" aria-label="Pulse">
      <a href="/xzhou99/dtalite_beta_test/pulse" aria-label="Pulse" class="js-selected-navigation-item sunken-menu-item" data-selected-links="pulse /xzhou99/dtalite_beta_test/pulse">
        <span class="octicon octicon-pulse"></span> <span class="full-word">Pulse</span>
        <img alt="" class="mini-loader" height="16" src="https://assets-cdn.github.com/images/spinners/octocat-spinner-32.gif" width="16" />
</a>    </li>

    <li class="tooltipped tooltipped-w" aria-label="Graphs">
      <a href="/xzhou99/dtalite_beta_test/graphs" aria-label="Graphs" class="js-selected-navigation-item sunken-menu-item" data-selected-links="repo_graphs repo_contributors /xzhou99/dtalite_beta_test/graphs">
        <span class="octicon octicon-graph"></span> <span class="full-word">Graphs</span>
        <img alt="" class="mini-loader" height="16" src="https://assets-cdn.github.com/images/spinners/octocat-spinner-32.gif" width="16" />
</a>    </li>
  </ul>


</nav>

            <div class="only-with-full-nav">
                
<div class="js-clone-url clone-url open"
  data-protocol-type="http">
  <h3><span class="text-emphasized">HTTPS</span> clone URL</h3>
  <div class="input-group js-zeroclipboard-container">
    <input type="text" class="input-mini input-monospace js-url-field js-zeroclipboard-target"
           value="https://github.com/xzhou99/dtalite_beta_test.git" readonly="readonly" aria-label="HTTPS clone URL">
    <span class="input-group-button">
      <button aria-label="Copy to clipboard" class="js-zeroclipboard btn btn-sm zeroclipboard-button tooltipped tooltipped-s" data-copied-hint="Copied!" type="button"><span class="octicon octicon-clippy"></span></button>
    </span>
  </div>
</div>

  
<div class="js-clone-url clone-url "
  data-protocol-type="ssh">
  <h3><span class="text-emphasized">SSH</span> clone URL</h3>
  <div class="input-group js-zeroclipboard-container">
    <input type="text" class="input-mini input-monospace js-url-field js-zeroclipboard-target"
           value="git@github.com:xzhou99/dtalite_beta_test.git" readonly="readonly" aria-label="SSH clone URL">
    <span class="input-group-button">
      <button aria-label="Copy to clipboard" class="js-zeroclipboard btn btn-sm zeroclipboard-button tooltipped tooltipped-s" data-copied-hint="Copied!" type="button"><span class="octicon octicon-clippy"></span></button>
    </span>
  </div>
</div>

  
<div class="js-clone-url clone-url "
  data-protocol-type="subversion">
  <h3><span class="text-emphasized">Subversion</span> checkout URL</h3>
  <div class="input-group js-zeroclipboard-container">
    <input type="text" class="input-mini input-monospace js-url-field js-zeroclipboard-target"
           value="https://github.com/xzhou99/dtalite_beta_test" readonly="readonly" aria-label="Subversion checkout URL">
    <span class="input-group-button">
      <button aria-label="Copy to clipboard" class="js-zeroclipboard btn btn-sm zeroclipboard-button tooltipped tooltipped-s" data-copied-hint="Copied!" type="button"><span class="octicon octicon-clippy"></span></button>
    </span>
  </div>
</div>



<div class="clone-options">You can clone with
  <!-- </textarea> --><!-- '"` --><form accept-charset="UTF-8" action="/users/set_protocol?protocol_selector=http&amp;protocol_type=clone" class="inline-form js-clone-selector-form is-enabled" data-form-nonce="a7483060e72f5a6ca485393f7ddb2f2d156361be" data-remote="true" method="post"><div style="margin:0;padding:0;display:inline"><input name="utf8" type="hidden" value="&#x2713;" /><input name="authenticity_token" type="hidden" value="bAgGQHqIn6zX/SsjAZzAwm2ngZNsjnm0/IXfcVhaBKCEKr3YtxbeJGWd5/Ob6i5U9R7c7xe4UJeL21F71XrIDQ==" /></div><button class="btn-link js-clone-selector" data-protocol="http" type="submit">HTTPS</button></form>, <!-- </textarea> --><!-- '"` --><form accept-charset="UTF-8" action="/users/set_protocol?protocol_selector=ssh&amp;protocol_type=clone" class="inline-form js-clone-selector-form is-enabled" data-form-nonce="a7483060e72f5a6ca485393f7ddb2f2d156361be" data-remote="true" method="post"><div style="margin:0;padding:0;display:inline"><input name="utf8" type="hidden" value="&#x2713;" /><input name="authenticity_token" type="hidden" value="T3fzlL8CDnfIznaAv+eEZLu90Rb2mfBiuwcZ+5OuV9leUlSRfj7IQaOKmOO+XzIemUd0jVPICAhOdVtE6dc+ZA==" /></div><button class="btn-link js-clone-selector" data-protocol="ssh" type="submit">SSH</button></form>, or <!-- </textarea> --><!-- '"` --><form accept-charset="UTF-8" action="/users/set_protocol?protocol_selector=subversion&amp;protocol_type=clone" class="inline-form js-clone-selector-form is-enabled" data-form-nonce="a7483060e72f5a6ca485393f7ddb2f2d156361be" data-remote="true" method="post"><div style="margin:0;padding:0;display:inline"><input name="utf8" type="hidden" value="&#x2713;" /><input name="authenticity_token" type="hidden" value="3JdTstLq3/KUlMUVLN+3HeOU+ibJW6GTC82Q2TlbLJGm0Qr86fs8QvMebRYUHyE5bcyFSCgo80s4f1qJSqR6gw==" /></div><button class="btn-link js-clone-selector" data-protocol="subversion" type="submit">Subversion</button></form>.
  <a href="https://help.github.com/articles/which-remote-url-should-i-use" class="help tooltipped tooltipped-n" aria-label="Get help on which URL is right for you.">
    <span class="octicon octicon-question"></span>
  </a>
</div>
  <a href="github-windows://openRepo/https://github.com/xzhou99/dtalite_beta_test" class="btn btn-sm sidebar-button" title="Save xzhou99/dtalite_beta_test to your computer and use it in GitHub Desktop." aria-label="Save xzhou99/dtalite_beta_test to your computer and use it in GitHub Desktop.">
    <span class="octicon octicon-desktop-download"></span>
    Clone in Desktop
  </a>

              <a href="/xzhou99/dtalite_beta_test/archive/master.zip"
                 class="btn btn-sm sidebar-button"
                 aria-label="Download the contents of xzhou99/dtalite_beta_test as a zip file"
                 title="Download the contents of xzhou99/dtalite_beta_test as a zip file"
                 rel="nofollow">
                <span class="octicon octicon-cloud-download"></span>
                Download ZIP
              </a>
            </div>
        </div>
        <div id="js-repo-pjax-container" class="repository-content context-loader-container" data-pjax-container>

          

<a href="/xzhou99/dtalite_beta_test/blob/57198ce93106b5adad014111b21983cce93919b9/Source_code_Version1_2/DTALite/EmissionsOutput.cpp" class="hidden js-permalink-shortcut" data-hotkey="y">Permalink</a>

<!-- blob contrib key: blob_contributors:v21:d0d7e3dfdcdd4d9e3e8c4470dce94892 -->

  <div class="file-navigation js-zeroclipboard-container">
    
<div class="select-menu js-menu-container js-select-menu left">
  <span class="btn btn-sm select-menu-button js-menu-target css-truncate" data-hotkey="w"
    title="master"
    role="button" aria-label="Switch branches or tags" tabindex="0" aria-haspopup="true">
    <i>Branch:</i>
    <span class="js-select-button css-truncate-target">master</span>
  </span>

  <div class="select-menu-modal-holder js-menu-content js-navigation-container" data-pjax aria-hidden="true">

    <div class="select-menu-modal">
      <div class="select-menu-header">
        <span class="select-menu-title">Switch branches/tags</span>
        <span class="octicon octicon-x js-menu-close" role="button" aria-label="Close"></span>
      </div>

      <div class="select-menu-filters">
        <div class="select-menu-text-filter">
          <input type="text" aria-label="Filter branches/tags" id="context-commitish-filter-field" class="js-filterable-field js-navigation-enable" placeholder="Filter branches/tags">
        </div>
        <div class="select-menu-tabs">
          <ul>
            <li class="select-menu-tab">
              <a href="#" data-tab-filter="branches" data-filter-placeholder="Filter branches/tags" class="js-select-menu-tab" role="tab">Branches</a>
            </li>
            <li class="select-menu-tab">
              <a href="#" data-tab-filter="tags" data-filter-placeholder="Find a tag…" class="js-select-menu-tab" role="tab">Tags</a>
            </li>
          </ul>
        </div>
      </div>

      <div class="select-menu-list select-menu-tab-bucket js-select-menu-tab-bucket" data-tab-filter="branches" role="menu">

        <div data-filterable-for="context-commitish-filter-field" data-filterable-type="substring">


            <a class="select-menu-item js-navigation-item js-navigation-open selected"
               href="/xzhou99/dtalite_beta_test/blob/master/Source_code_Version1_2/DTALite/EmissionsOutput.cpp"
               data-name="master"
               data-skip-pjax="true"
               rel="nofollow">
              <span class="select-menu-item-icon octicon octicon-check"></span>
              <span class="select-menu-item-text css-truncate-target" title="master">
                master
              </span>
            </a>
        </div>

          <div class="select-menu-no-results">Nothing to show</div>
      </div>

      <div class="select-menu-list select-menu-tab-bucket js-select-menu-tab-bucket" data-tab-filter="tags">
        <div data-filterable-for="context-commitish-filter-field" data-filterable-type="substring">


        </div>

        <div class="select-menu-no-results">Nothing to show</div>
      </div>

    </div>
  </div>
</div>

    <div class="btn-group right">
      <a href="/xzhou99/dtalite_beta_test/find/master"
            class="js-show-file-finder btn btn-sm empty-icon tooltipped tooltipped-nw"
            data-pjax
            data-hotkey="t"
            aria-label="Quickly jump between files">
        <span class="octicon octicon-list-unordered"></span>
      </a>
      <button aria-label="Copy file path to clipboard" class="js-zeroclipboard btn btn-sm zeroclipboard-button tooltipped tooltipped-s" data-copied-hint="Copied!" type="button"><span class="octicon octicon-clippy"></span></button>
    </div>

    <div class="breadcrumb js-zeroclipboard-target">
      <span class="repo-root js-repo-root"><span itemscope="" itemtype="http://data-vocabulary.org/Breadcrumb"><a href="/xzhou99/dtalite_beta_test" class="" data-branch="master" data-pjax="true" itemscope="url"><span itemprop="title">dtalite_beta_test</span></a></span></span><span class="separator">/</span><span itemscope="" itemtype="http://data-vocabulary.org/Breadcrumb"><a href="/xzhou99/dtalite_beta_test/tree/master/Source_code_Version1_2" class="" data-branch="master" data-pjax="true" itemscope="url"><span itemprop="title">Source_code_Version1_2</span></a></span><span class="separator">/</span><span itemscope="" itemtype="http://data-vocabulary.org/Breadcrumb"><a href="/xzhou99/dtalite_beta_test/tree/master/Source_code_Version1_2/DTALite" class="" data-branch="master" data-pjax="true" itemscope="url"><span itemprop="title">DTALite</span></a></span><span class="separator">/</span><strong class="final-path">EmissionsOutput.cpp</strong>
    </div>
  </div>


  <div class="commit file-history-tease">
    <div class="file-history-tease-header">
        <img alt="@xzhou99" class="avatar" height="24" src="https://avatars0.githubusercontent.com/u/3403345?v=3&amp;s=48" width="24" />
        <span class="author"><a href="/xzhou99" rel="author">xzhou99</a></span>
        <time datetime="2015-09-27T21:31:26Z" is="relative-time">Sep 27, 2015</time>
        <div class="commit-title">
            <a href="/xzhou99/dtalite_beta_test/commit/9137ef58c44c6abbdf76b706adeb2e6aaa57acd7" class="message" data-pjax="true" title="emission debugging code

working with Shams to add emission debugging code">emission debugging code</a>
        </div>
    </div>

    <div class="participation">
      <p class="quickstat">
        <a href="#blob_contributors_box" rel="facebox">
          <strong>1</strong>
           contributor
        </a>
      </p>
      
    </div>
    <div id="blob_contributors_box" style="display:none">
      <h2 class="facebox-header" data-facebox-id="facebox-header">Users who have contributed to this file</h2>
      <ul class="facebox-user-list" data-facebox-id="facebox-description">
          <li class="facebox-user-list-item">
            <img alt="@xzhou99" height="24" src="https://avatars0.githubusercontent.com/u/3403345?v=3&amp;s=48" width="24" />
            <a href="/xzhou99">xzhou99</a>
          </li>
      </ul>
    </div>
  </div>

<div class="file">
  <div class="file-header">
  <div class="file-actions">

    <div class="btn-group">
      <a href="/xzhou99/dtalite_beta_test/raw/master/Source_code_Version1_2/DTALite/EmissionsOutput.cpp" class="btn btn-sm " id="raw-url">Raw</a>
        <a href="/xzhou99/dtalite_beta_test/blame/master/Source_code_Version1_2/DTALite/EmissionsOutput.cpp" class="btn btn-sm js-update-url-with-hash">Blame</a>
      <a href="/xzhou99/dtalite_beta_test/commits/master/Source_code_Version1_2/DTALite/EmissionsOutput.cpp" class="btn btn-sm " rel="nofollow">History</a>
    </div>

      <a class="octicon-btn tooltipped tooltipped-nw"
         href="github-windows://openRepo/https://github.com/xzhou99/dtalite_beta_test?branch=master&amp;filepath=Source_code_Version1_2%2FDTALite%2FEmissionsOutput.cpp"
         aria-label="Open this file in GitHub Desktop"
         data-ga-click="Repository, open with desktop, type:windows">
          <span class="octicon octicon-device-desktop"></span>
      </a>

        <!-- </textarea> --><!-- '"` --><form accept-charset="UTF-8" action="/xzhou99/dtalite_beta_test/edit/master/Source_code_Version1_2/DTALite/EmissionsOutput.cpp" class="inline-form js-update-url-with-hash" data-form-nonce="a7483060e72f5a6ca485393f7ddb2f2d156361be" method="post"><div style="margin:0;padding:0;display:inline"><input name="utf8" type="hidden" value="&#x2713;" /><input name="authenticity_token" type="hidden" value="9VzqoRME60u6A5BXDfZZ8QAdSaaahbQa9Rhiizk8dyL3oL0DLwzgEvYkAgE3siPQV1zYUYUPCHo6+avGeinkNQ==" /></div>
          <button class="octicon-btn tooltipped tooltipped-nw" type="submit"
            aria-label="Edit the file in your fork of this project" data-hotkey="e" data-disable-with>
            <span class="octicon octicon-pencil"></span>
          </button>
</form>        <!-- </textarea> --><!-- '"` --><form accept-charset="UTF-8" action="/xzhou99/dtalite_beta_test/delete/master/Source_code_Version1_2/DTALite/EmissionsOutput.cpp" class="inline-form" data-form-nonce="a7483060e72f5a6ca485393f7ddb2f2d156361be" method="post"><div style="margin:0;padding:0;display:inline"><input name="utf8" type="hidden" value="&#x2713;" /><input name="authenticity_token" type="hidden" value="ZJqN0KimH+hexbMTnNWbzy8/2buYreVxufLvY4bd2aDpzJMVryrHu9bhdiMaVOkWYEHZ0jOrvDnCXXoUcRL9Vw==" /></div>
          <button class="octicon-btn octicon-btn-danger tooltipped tooltipped-nw" type="submit"
            aria-label="Delete the file in your fork of this project" data-disable-with>
            <span class="octicon octicon-trashcan"></span>
          </button>
</form>  </div>

  <div class="file-info">
      3090 lines (2512 sloc)
      <span class="file-info-divider"></span>
    217 KB
  </div>
</div>

  

  <div class="blob-wrapper data type-c">
      <table class="highlight tab-size js-file-line-container" data-tab-size="8">
      <tr>
        <td id="L1" class="blob-num js-line-number" data-line-number="1"></td>
        <td id="LC1" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//  Portions Copyright 2010 Xuesong Zhou, Hao Lei</span></td>
      </tr>
      <tr>
        <td id="L2" class="blob-num js-line-number" data-line-number="2"></td>
        <td id="LC2" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3" class="blob-num js-line-number" data-line-number="3"></td>
        <td id="LC3" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//   If you help write or modify the code, please also list your names here.</span></td>
      </tr>
      <tr>
        <td id="L4" class="blob-num js-line-number" data-line-number="4"></td>
        <td id="LC4" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//   The reason of having copyright info here is to ensure all the modified version, as a whole, under the GPL </span></td>
      </tr>
      <tr>
        <td id="L5" class="blob-num js-line-number" data-line-number="5"></td>
        <td id="LC5" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//   and further prevent a violation of the GPL.</span></td>
      </tr>
      <tr>
        <td id="L6" class="blob-num js-line-number" data-line-number="6"></td>
        <td id="LC6" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L7" class="blob-num js-line-number" data-line-number="7"></td>
        <td id="LC7" class="blob-code blob-code-inner js-file-line"><span class="pl-c">// More about &quot;How to use GNU licenses for your own software&quot;</span></td>
      </tr>
      <tr>
        <td id="L8" class="blob-num js-line-number" data-line-number="8"></td>
        <td id="LC8" class="blob-code blob-code-inner js-file-line"><span class="pl-c">// http://www.gnu.org/licenses/gpl-howto.html</span></td>
      </tr>
      <tr>
        <td id="L9" class="blob-num js-line-number" data-line-number="9"></td>
        <td id="LC9" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L10" class="blob-num js-line-number" data-line-number="10"></td>
        <td id="LC10" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//    This file is part of DTALite.</span></td>
      </tr>
      <tr>
        <td id="L11" class="blob-num js-line-number" data-line-number="11"></td>
        <td id="LC11" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L12" class="blob-num js-line-number" data-line-number="12"></td>
        <td id="LC12" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//    DTALite is free software: you can redistribute it and/or modify</span></td>
      </tr>
      <tr>
        <td id="L13" class="blob-num js-line-number" data-line-number="13"></td>
        <td id="LC13" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//    it under the terms of the GNU General Public License as published by</span></td>
      </tr>
      <tr>
        <td id="L14" class="blob-num js-line-number" data-line-number="14"></td>
        <td id="LC14" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//    the Free Software Foundation, either version 3 of the License, or</span></td>
      </tr>
      <tr>
        <td id="L15" class="blob-num js-line-number" data-line-number="15"></td>
        <td id="LC15" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//    (at your option) any later version.</span></td>
      </tr>
      <tr>
        <td id="L16" class="blob-num js-line-number" data-line-number="16"></td>
        <td id="LC16" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L17" class="blob-num js-line-number" data-line-number="17"></td>
        <td id="LC17" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//    DTALite is distributed in the hope that it will be useful,</span></td>
      </tr>
      <tr>
        <td id="L18" class="blob-num js-line-number" data-line-number="18"></td>
        <td id="LC18" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//    but WITHOUT ANY WARRANTY; without even the implied warranty of</span></td>
      </tr>
      <tr>
        <td id="L19" class="blob-num js-line-number" data-line-number="19"></td>
        <td id="LC19" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the</span></td>
      </tr>
      <tr>
        <td id="L20" class="blob-num js-line-number" data-line-number="20"></td>
        <td id="LC20" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//    GNU General Public License for more details.</span></td>
      </tr>
      <tr>
        <td id="L21" class="blob-num js-line-number" data-line-number="21"></td>
        <td id="LC21" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L22" class="blob-num js-line-number" data-line-number="22"></td>
        <td id="LC22" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//    You should have received a copy of the GNU General Public License</span></td>
      </tr>
      <tr>
        <td id="L23" class="blob-num js-line-number" data-line-number="23"></td>
        <td id="LC23" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//    along with DTALite.  If not, see &lt;http://www.gnu.org/licenses/&gt;.</span></td>
      </tr>
      <tr>
        <td id="L24" class="blob-num js-line-number" data-line-number="24"></td>
        <td id="LC24" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L25" class="blob-num js-line-number" data-line-number="25"></td>
        <td id="LC25" class="blob-code blob-code-inner js-file-line"><span class="pl-c">// DTALite.cpp : Defines the entry point for the console application.</span></td>
      </tr>
      <tr>
        <td id="L26" class="blob-num js-line-number" data-line-number="26"></td>
        <td id="LC26" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L27" class="blob-num js-line-number" data-line-number="27"></td>
        <td id="LC27" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">include</span> <span class="pl-s"><span class="pl-pds">&quot;</span>stdafx.h<span class="pl-pds">&quot;</span></span></td>
      </tr>
      <tr>
        <td id="L28" class="blob-num js-line-number" data-line-number="28"></td>
        <td id="LC28" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">include</span> <span class="pl-s"><span class="pl-pds">&quot;</span>DTALite.h<span class="pl-pds">&quot;</span></span></td>
      </tr>
      <tr>
        <td id="L29" class="blob-num js-line-number" data-line-number="29"></td>
        <td id="LC29" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">include</span> <span class="pl-s"><span class="pl-pds">&quot;</span>GlobalData.h<span class="pl-pds">&quot;</span></span></td>
      </tr>
      <tr>
        <td id="L30" class="blob-num js-line-number" data-line-number="30"></td>
        <td id="LC30" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">include</span> <span class="pl-s"><span class="pl-pds">&quot;</span>CSVParser.h<span class="pl-pds">&quot;</span></span></td>
      </tr>
      <tr>
        <td id="L31" class="blob-num js-line-number" data-line-number="31"></td>
        <td id="LC31" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L32" class="blob-num js-line-number" data-line-number="32"></td>
        <td id="LC32" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">include</span> <span class="pl-s"><span class="pl-pds">&lt;</span>iostream<span class="pl-pds">&gt;</span></span></td>
      </tr>
      <tr>
        <td id="L33" class="blob-num js-line-number" data-line-number="33"></td>
        <td id="LC33" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">include</span> <span class="pl-s"><span class="pl-pds">&lt;</span>fstream<span class="pl-pds">&gt;</span></span></td>
      </tr>
      <tr>
        <td id="L34" class="blob-num js-line-number" data-line-number="34"></td>
        <td id="LC34" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">include</span> <span class="pl-s"><span class="pl-pds">&lt;</span>omp.h<span class="pl-pds">&gt;</span></span></td>
      </tr>
      <tr>
        <td id="L35" class="blob-num js-line-number" data-line-number="35"></td>
        <td id="LC35" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">include</span> <span class="pl-s"><span class="pl-pds">&lt;</span>algorithm<span class="pl-pds">&gt;</span></span></td>
      </tr>
      <tr>
        <td id="L36" class="blob-num js-line-number" data-line-number="36"></td>
        <td id="LC36" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L37" class="blob-num js-line-number" data-line-number="37"></td>
        <td id="LC37" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L38" class="blob-num js-line-number" data-line-number="38"></td>
        <td id="LC38" class="blob-code blob-code-inner js-file-line"><span class="pl-k">using</span> <span class="pl-k">namespace</span> <span class="pl-en">std</span><span class="pl-k">;</span></td>
      </tr>
      <tr>
        <td id="L39" class="blob-num js-line-number" data-line-number="39"></td>
        <td id="LC39" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L40" class="blob-num js-line-number" data-line-number="40"></td>
        <td id="LC40" class="blob-code blob-code-inner js-file-line"><span class="pl-k">const</span> <span class="pl-k">int</span> NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND = <span class="pl-c1">10</span>;</td>
      </tr>
      <tr>
        <td id="L41" class="blob-num js-line-number" data-line-number="41"></td>
        <td id="LC41" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">_MAXIMUM_OPERATING_MODE_SIZE</span> <span class="pl-c1">41</span></td>
      </tr>
      <tr>
        <td id="L42" class="blob-num js-line-number" data-line-number="42"></td>
        <td id="LC42" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">define</span> <span class="pl-en">_MAXIMUM_AGE_SIZE</span> <span class="pl-c1">31</span></td>
      </tr>
      <tr>
        <td id="L43" class="blob-num js-line-number" data-line-number="43"></td>
        <td id="LC43" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L44" class="blob-num js-line-number" data-line-number="44"></td>
        <td id="LC44" class="blob-code blob-code-inner js-file-line"><span class="pl-k">int</span> OperatingModeMap[MAX_SPEED_BIN][MAX_VSP_BIN] = {-<span class="pl-c1">1</span>};</td>
      </tr>
      <tr>
        <td id="L45" class="blob-num js-line-number" data-line-number="45"></td>
        <td id="LC45" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L46" class="blob-num js-line-number" data-line-number="46"></td>
        <td id="LC46" class="blob-code blob-code-inner js-file-line"><span class="pl-k">class</span> <span class="pl-en">CEmissionRate</span> </td>
      </tr>
      <tr>
        <td id="L47" class="blob-num js-line-number" data-line-number="47"></td>
        <td id="LC47" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L48" class="blob-num js-line-number" data-line-number="48"></td>
        <td id="LC48" class="blob-code blob-code-inner js-file-line"><span class="pl-k">public:</span></td>
      </tr>
      <tr>
        <td id="L49" class="blob-num js-line-number" data-line-number="49"></td>
        <td id="LC49" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">bool</span> bInitialized;</td>
      </tr>
      <tr>
        <td id="L50" class="blob-num js-line-number" data-line-number="50"></td>
        <td id="LC50" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> meanBaseRate_TotalEnergy;</td>
      </tr>
      <tr>
        <td id="L51" class="blob-num js-line-number" data-line-number="51"></td>
        <td id="LC51" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> meanBaseRate_CO2;</td>
      </tr>
      <tr>
        <td id="L52" class="blob-num js-line-number" data-line-number="52"></td>
        <td id="LC52" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> meanBaseRate_NOX;</td>
      </tr>
      <tr>
        <td id="L53" class="blob-num js-line-number" data-line-number="53"></td>
        <td id="LC53" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> meanBaseRate_CO;</td>
      </tr>
      <tr>
        <td id="L54" class="blob-num js-line-number" data-line-number="54"></td>
        <td id="LC54" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> meanBaseRate_HC;</td>
      </tr>
      <tr>
        <td id="L55" class="blob-num js-line-number" data-line-number="55"></td>
        <td id="LC55" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> Age;</td>
      </tr>
      <tr>
        <td id="L56" class="blob-num js-line-number" data-line-number="56"></td>
        <td id="LC56" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L57" class="blob-num js-line-number" data-line-number="57"></td>
        <td id="LC57" class="blob-code blob-code-inner js-file-line">	<span class="pl-en">CEmissionRate</span>()</td>
      </tr>
      <tr>
        <td id="L58" class="blob-num js-line-number" data-line-number="58"></td>
        <td id="LC58" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L59" class="blob-num js-line-number" data-line-number="59"></td>
        <td id="LC59" class="blob-code blob-code-inner js-file-line">		bInitialized = <span class="pl-c1">false</span>;</td>
      </tr>
      <tr>
        <td id="L60" class="blob-num js-line-number" data-line-number="60"></td>
        <td id="LC60" class="blob-code blob-code-inner js-file-line">		meanBaseRate_TotalEnergy = meanBaseRate_CO2 = meanBaseRate_NOX = meanBaseRate_CO = meanBaseRate_HC = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L61" class="blob-num js-line-number" data-line-number="61"></td>
        <td id="LC61" class="blob-code blob-code-inner js-file-line">		Age = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L62" class="blob-num js-line-number" data-line-number="62"></td>
        <td id="LC62" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L63" class="blob-num js-line-number" data-line-number="63"></td>
        <td id="LC63" class="blob-code blob-code-inner js-file-line">};</td>
      </tr>
      <tr>
        <td id="L64" class="blob-num js-line-number" data-line-number="64"></td>
        <td id="LC64" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L65" class="blob-num js-line-number" data-line-number="65"></td>
        <td id="LC65" class="blob-code blob-code-inner js-file-line">CEmissionRate EmissionRateData[MAX_VEHICLE_TYPE_SIZE][_MAXIMUM_OPERATING_MODE_SIZE][_MAXIMUM_AGE_SIZE];</td>
      </tr>
      <tr>
        <td id="L66" class="blob-num js-line-number" data-line-number="66"></td>
        <td id="LC66" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L67" class="blob-num js-line-number" data-line-number="67"></td>
        <td id="LC67" class="blob-code blob-code-inner js-file-line"><span class="pl-k">class</span> <span class="pl-en">CCycleAverageEmissionFactor</span></td>
      </tr>
      <tr>
        <td id="L68" class="blob-num js-line-number" data-line-number="68"></td>
        <td id="LC68" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L69" class="blob-num js-line-number" data-line-number="69"></td>
        <td id="LC69" class="blob-code blob-code-inner js-file-line"><span class="pl-k">public:</span></td>
      </tr>
      <tr>
        <td id="L70" class="blob-num js-line-number" data-line-number="70"></td>
        <td id="LC70" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> emissionFactor_CO2;</td>
      </tr>
      <tr>
        <td id="L71" class="blob-num js-line-number" data-line-number="71"></td>
        <td id="LC71" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> emissionFactor_NOX;</td>
      </tr>
      <tr>
        <td id="L72" class="blob-num js-line-number" data-line-number="72"></td>
        <td id="LC72" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> emissionFactor_CO;</td>
      </tr>
      <tr>
        <td id="L73" class="blob-num js-line-number" data-line-number="73"></td>
        <td id="LC73" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> emissionFactor_HC;</td>
      </tr>
      <tr>
        <td id="L74" class="blob-num js-line-number" data-line-number="74"></td>
        <td id="LC74" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> average_cycle_speed;</td>
      </tr>
      <tr>
        <td id="L75" class="blob-num js-line-number" data-line-number="75"></td>
        <td id="LC75" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L76" class="blob-num js-line-number" data-line-number="76"></td>
        <td id="LC76" class="blob-code blob-code-inner js-file-line">	<span class="pl-en">CCycleAverageEmissionFactor</span>()</td>
      </tr>
      <tr>
        <td id="L77" class="blob-num js-line-number" data-line-number="77"></td>
        <td id="LC77" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L78" class="blob-num js-line-number" data-line-number="78"></td>
        <td id="LC78" class="blob-code blob-code-inner js-file-line">		<span class="pl-v">this</span>-&gt;emissionFactor_CO2 = <span class="pl-v">this</span>-&gt;emissionFactor_NOX = <span class="pl-v">this</span>-&gt;emissionFactor_CO = <span class="pl-v">this</span>-&gt;emissionFactor_HC = <span class="pl-c1">0</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L79" class="blob-num js-line-number" data-line-number="79"></td>
        <td id="LC79" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L80" class="blob-num js-line-number" data-line-number="80"></td>
        <td id="LC80" class="blob-code blob-code-inner js-file-line">	<span class="pl-en">CCycleAverageEmissionFactor</span>(<span class="pl-k">float</span> emissionFactor_CO2, <span class="pl-k">float</span> emissionFactor_NOX, <span class="pl-k">float</span> emissionFactor_CO, <span class="pl-k">float</span> emissionFactor_HC)</td>
      </tr>
      <tr>
        <td id="L81" class="blob-num js-line-number" data-line-number="81"></td>
        <td id="LC81" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L82" class="blob-num js-line-number" data-line-number="82"></td>
        <td id="LC82" class="blob-code blob-code-inner js-file-line">		<span class="pl-v">this</span>-&gt;emissionFactor_CO2 = emissionFactor_CO2;</td>
      </tr>
      <tr>
        <td id="L83" class="blob-num js-line-number" data-line-number="83"></td>
        <td id="LC83" class="blob-code blob-code-inner js-file-line">		<span class="pl-v">this</span>-&gt;emissionFactor_NOX = emissionFactor_NOX;</td>
      </tr>
      <tr>
        <td id="L84" class="blob-num js-line-number" data-line-number="84"></td>
        <td id="LC84" class="blob-code blob-code-inner js-file-line">		<span class="pl-v">this</span>-&gt;emissionFactor_CO = emissionFactor_CO;</td>
      </tr>
      <tr>
        <td id="L85" class="blob-num js-line-number" data-line-number="85"></td>
        <td id="LC85" class="blob-code blob-code-inner js-file-line">		<span class="pl-v">this</span>-&gt;emissionFactor_HC = emissionFactor_HC;</td>
      </tr>
      <tr>
        <td id="L86" class="blob-num js-line-number" data-line-number="86"></td>
        <td id="LC86" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L87" class="blob-num js-line-number" data-line-number="87"></td>
        <td id="LC87" class="blob-code blob-code-inner js-file-line">};</td>
      </tr>
      <tr>
        <td id="L88" class="blob-num js-line-number" data-line-number="88"></td>
        <td id="LC88" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L89" class="blob-num js-line-number" data-line-number="89"></td>
        <td id="LC89" class="blob-code blob-code-inner js-file-line">CCycleAverageEmissionFactor CycleAverageEmissionFactorMatrix[MAX_VEHICLE_TYPE_SIZE][_MAXIMUM_AGE_SIZE];</td>
      </tr>
      <tr>
        <td id="L90" class="blob-num js-line-number" data-line-number="90"></td>
        <td id="LC90" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L91" class="blob-num js-line-number" data-line-number="91"></td>
        <td id="LC91" class="blob-code blob-code-inner js-file-line"><span class="pl-k">class</span> <span class="pl-en">CVehicleEmissionResult</span></td>
      </tr>
      <tr>
        <td id="L92" class="blob-num js-line-number" data-line-number="92"></td>
        <td id="LC92" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L93" class="blob-num js-line-number" data-line-number="93"></td>
        <td id="LC93" class="blob-code blob-code-inner js-file-line"><span class="pl-k">public:</span></td>
      </tr>
      <tr>
        <td id="L94" class="blob-num js-line-number" data-line-number="94"></td>
        <td id="LC94" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> Energy;</td>
      </tr>
      <tr>
        <td id="L95" class="blob-num js-line-number" data-line-number="95"></td>
        <td id="LC95" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> CO2;</td>
      </tr>
      <tr>
        <td id="L96" class="blob-num js-line-number" data-line-number="96"></td>
        <td id="LC96" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> NOX;</td>
      </tr>
      <tr>
        <td id="L97" class="blob-num js-line-number" data-line-number="97"></td>
        <td id="LC97" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> CO;</td>
      </tr>
      <tr>
        <td id="L98" class="blob-num js-line-number" data-line-number="98"></td>
        <td id="LC98" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> HC;</td>
      </tr>
      <tr>
        <td id="L99" class="blob-num js-line-number" data-line-number="99"></td>
        <td id="LC99" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L100" class="blob-num js-line-number" data-line-number="100"></td>
        <td id="LC100" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">static</span> <span class="pl-k">void</span> <span class="pl-en">CalculateEmissions</span>(<span class="pl-k">int</span> vehicle_type, std::map&lt;<span class="pl-k">int</span>, <span class="pl-k">int</span>&gt;&amp; OperatingModeCount, <span class="pl-k">int</span> age, CVehicleEmissionResult&amp; emissionResult)</td>
      </tr>
      <tr>
        <td id="L101" class="blob-num js-line-number" data-line-number="101"></td>
        <td id="LC101" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L102" class="blob-num js-line-number" data-line-number="102"></td>
        <td id="LC102" class="blob-code blob-code-inner js-file-line">		emissionResult.<span class="pl-smi">Energy</span> = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L103" class="blob-num js-line-number" data-line-number="103"></td>
        <td id="LC103" class="blob-code blob-code-inner js-file-line">		emissionResult.<span class="pl-smi">CO2</span> = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L104" class="blob-num js-line-number" data-line-number="104"></td>
        <td id="LC104" class="blob-code blob-code-inner js-file-line">		emissionResult.<span class="pl-smi">NOX</span> = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L105" class="blob-num js-line-number" data-line-number="105"></td>
        <td id="LC105" class="blob-code blob-code-inner js-file-line">		emissionResult.<span class="pl-smi">CO</span> = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L106" class="blob-num js-line-number" data-line-number="106"></td>
        <td id="LC106" class="blob-code blob-code-inner js-file-line">		emissionResult.<span class="pl-smi">HC</span> = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L107" class="blob-num js-line-number" data-line-number="107"></td>
        <td id="LC107" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L108" class="blob-num js-line-number" data-line-number="108"></td>
        <td id="LC108" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">for</span>(std::map&lt;<span class="pl-k">int</span>, <span class="pl-k">int</span>&gt;::iterator iterOP  = OperatingModeCount.<span class="pl-c1">begin</span>(); iterOP != OperatingModeCount.<span class="pl-c1">end</span> (); iterOP++)</td>
      </tr>
      <tr>
        <td id="L109" class="blob-num js-line-number" data-line-number="109"></td>
        <td id="LC109" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L110" class="blob-num js-line-number" data-line-number="110"></td>
        <td id="LC110" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">int</span> OpModeID = iterOP-&gt;first; <span class="pl-k">int</span> count = iterOP-&gt;second;</td>
      </tr>
      <tr>
        <td id="L111" class="blob-num js-line-number" data-line-number="111"></td>
        <td id="LC111" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L112" class="blob-num js-line-number" data-line-number="112"></td>
        <td id="LC112" class="blob-code blob-code-inner js-file-line">			emissionResult.<span class="pl-smi">Energy</span>+= EmissionRateData[vehicle_type][OpModeID][age].<span class="pl-smi">meanBaseRate_TotalEnergy</span>*count/<span class="pl-c1">3600</span>;</td>
      </tr>
      <tr>
        <td id="L113" class="blob-num js-line-number" data-line-number="113"></td>
        <td id="LC113" class="blob-code blob-code-inner js-file-line">			emissionResult.<span class="pl-smi">CO2</span>+= EmissionRateData[vehicle_type][OpModeID][age].<span class="pl-smi">meanBaseRate_CO2</span>*count/<span class="pl-c1">3600</span>;</td>
      </tr>
      <tr>
        <td id="L114" class="blob-num js-line-number" data-line-number="114"></td>
        <td id="LC114" class="blob-code blob-code-inner js-file-line">			emissionResult.<span class="pl-smi">NOX</span>+= EmissionRateData[vehicle_type][OpModeID][age].<span class="pl-smi">meanBaseRate_NOX</span>*count/<span class="pl-c1">3600</span>;</td>
      </tr>
      <tr>
        <td id="L115" class="blob-num js-line-number" data-line-number="115"></td>
        <td id="LC115" class="blob-code blob-code-inner js-file-line">			emissionResult.<span class="pl-smi">CO</span>+= EmissionRateData[vehicle_type][OpModeID][age].<span class="pl-smi">meanBaseRate_CO</span>*count/<span class="pl-c1">3600</span>;</td>
      </tr>
      <tr>
        <td id="L116" class="blob-num js-line-number" data-line-number="116"></td>
        <td id="LC116" class="blob-code blob-code-inner js-file-line">			emissionResult.<span class="pl-smi">HC</span>+= EmissionRateData[vehicle_type][OpModeID][age].<span class="pl-smi">meanBaseRate_HC</span>*count/<span class="pl-c1">3600</span>;</td>
      </tr>
      <tr>
        <td id="L117" class="blob-num js-line-number" data-line-number="117"></td>
        <td id="LC117" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L118" class="blob-num js-line-number" data-line-number="118"></td>
        <td id="LC118" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L119" class="blob-num js-line-number" data-line-number="119"></td>
        <td id="LC119" class="blob-code blob-code-inner js-file-line">};</td>
      </tr>
      <tr>
        <td id="L120" class="blob-num js-line-number" data-line-number="120"></td>
        <td id="LC120" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L121" class="blob-num js-line-number" data-line-number="121"></td>
        <td id="LC121" class="blob-code blob-code-inner js-file-line">SPEED_BIN <span class="pl-en">GetSpeedBinNo</span>(<span class="pl-k">float</span> speed_mph)</td>
      </tr>
      <tr>
        <td id="L122" class="blob-num js-line-number" data-line-number="122"></td>
        <td id="LC122" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L123" class="blob-num js-line-number" data-line-number="123"></td>
        <td id="LC123" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//the precision of speed output is 2</span></td>
      </tr>
      <tr>
        <td id="L124" class="blob-num js-line-number" data-line-number="124"></td>
        <td id="LC124" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//if the speed is 49.995, it would round up to 50</span></td>
      </tr>
      <tr>
        <td id="L125" class="blob-num js-line-number" data-line-number="125"></td>
        <td id="LC125" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//if the speed is 49.994, it would round to 49.99</span></td>
      </tr>
      <tr>
        <td id="L126" class="blob-num js-line-number" data-line-number="126"></td>
        <td id="LC126" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> speed = <span class="pl-c1">floor</span>(speed_mph + <span class="pl-c1">0.005</span>);</td>
      </tr>
      <tr>
        <td id="L127" class="blob-num js-line-number" data-line-number="127"></td>
        <td id="LC127" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L128" class="blob-num js-line-number" data-line-number="128"></td>
        <td id="LC128" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span>(speed &lt;= <span class="pl-c1">25</span>)</td>
      </tr>
      <tr>
        <td id="L129" class="blob-num js-line-number" data-line-number="129"></td>
        <td id="LC129" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">return</span> VSP_0_25mph;</td>
      </tr>
      <tr>
        <td id="L130" class="blob-num js-line-number" data-line-number="130"></td>
        <td id="LC130" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L131" class="blob-num js-line-number" data-line-number="131"></td>
        <td id="LC131" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span>(speed &lt; <span class="pl-c1">50</span>)</td>
      </tr>
      <tr>
        <td id="L132" class="blob-num js-line-number" data-line-number="132"></td>
        <td id="LC132" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">return</span> VSP_25_50mph;</td>
      </tr>
      <tr>
        <td id="L133" class="blob-num js-line-number" data-line-number="133"></td>
        <td id="LC133" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">else</span> </td>
      </tr>
      <tr>
        <td id="L134" class="blob-num js-line-number" data-line-number="134"></td>
        <td id="LC134" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">return</span> VSP_GT50mph;</td>
      </tr>
      <tr>
        <td id="L135" class="blob-num js-line-number" data-line-number="135"></td>
        <td id="LC135" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L136" class="blob-num js-line-number" data-line-number="136"></td>
        <td id="LC136" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L137" class="blob-num js-line-number" data-line-number="137"></td>
        <td id="LC137" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L138" class="blob-num js-line-number" data-line-number="138"></td>
        <td id="LC138" class="blob-code blob-code-inner js-file-line">VSP_BIN <span class="pl-en">GetVSPBinNo</span>(<span class="pl-k">float</span> vsp, <span class="pl-k">float</span> speed_mph)</td>
      </tr>
      <tr>
        <td id="L139" class="blob-num js-line-number" data-line-number="139"></td>
        <td id="LC139" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L140" class="blob-num js-line-number" data-line-number="140"></td>
        <td id="LC140" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//if(speed_mph &lt; 50)</span></td>
      </tr>
      <tr>
        <td id="L141" class="blob-num js-line-number" data-line-number="141"></td>
        <td id="LC141" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//{</span></td>
      </tr>
      <tr>
        <td id="L142" class="blob-num js-line-number" data-line-number="142"></td>
        <td id="LC142" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (vsp &lt; <span class="pl-c1">0</span>) <span class="pl-k">return</span> VSP_LT0;</td>
      </tr>
      <tr>
        <td id="L143" class="blob-num js-line-number" data-line-number="143"></td>
        <td id="LC143" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L144" class="blob-num js-line-number" data-line-number="144"></td>
        <td id="LC144" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (vsp &lt; <span class="pl-c1">3</span>) <span class="pl-k">return</span> VSP_0_3;</td>
      </tr>
      <tr>
        <td id="L145" class="blob-num js-line-number" data-line-number="145"></td>
        <td id="LC145" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L146" class="blob-num js-line-number" data-line-number="146"></td>
        <td id="LC146" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (vsp &lt; <span class="pl-c1">6</span>) <span class="pl-k">return</span> VSP_3_6;</td>
      </tr>
      <tr>
        <td id="L147" class="blob-num js-line-number" data-line-number="147"></td>
        <td id="LC147" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L148" class="blob-num js-line-number" data-line-number="148"></td>
        <td id="LC148" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (vsp &lt; <span class="pl-c1">9</span>) <span class="pl-k">return</span> VSP_6_9;</td>
      </tr>
      <tr>
        <td id="L149" class="blob-num js-line-number" data-line-number="149"></td>
        <td id="LC149" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L150" class="blob-num js-line-number" data-line-number="150"></td>
        <td id="LC150" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (vsp &lt; <span class="pl-c1">12</span>) <span class="pl-k">return</span> VSP_9_12;</td>
      </tr>
      <tr>
        <td id="L151" class="blob-num js-line-number" data-line-number="151"></td>
        <td id="LC151" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L152" class="blob-num js-line-number" data-line-number="152"></td>
        <td id="LC152" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (vsp &lt; <span class="pl-c1">18</span>) <span class="pl-k">return</span> VSP_12_18;</td>
      </tr>
      <tr>
        <td id="L153" class="blob-num js-line-number" data-line-number="153"></td>
        <td id="LC153" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L154" class="blob-num js-line-number" data-line-number="154"></td>
        <td id="LC154" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (vsp &lt; <span class="pl-c1">24</span>) <span class="pl-k">return</span> VSP_18_24;</td>
      </tr>
      <tr>
        <td id="L155" class="blob-num js-line-number" data-line-number="155"></td>
        <td id="LC155" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L156" class="blob-num js-line-number" data-line-number="156"></td>
        <td id="LC156" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (vsp &lt; <span class="pl-c1">30</span>) <span class="pl-k">return</span> VSP_24_30;</td>
      </tr>
      <tr>
        <td id="L157" class="blob-num js-line-number" data-line-number="157"></td>
        <td id="LC157" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L158" class="blob-num js-line-number" data-line-number="158"></td>
        <td id="LC158" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">return</span> VSP_GT30;</td>
      </tr>
      <tr>
        <td id="L159" class="blob-num js-line-number" data-line-number="159"></td>
        <td id="LC159" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L160" class="blob-num js-line-number" data-line-number="160"></td>
        <td id="LC160" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//}</span></td>
      </tr>
      <tr>
        <td id="L161" class="blob-num js-line-number" data-line-number="161"></td>
        <td id="LC161" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//else  // greate than 50</span></td>
      </tr>
      <tr>
        <td id="L162" class="blob-num js-line-number" data-line-number="162"></td>
        <td id="LC162" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//{</span></td>
      </tr>
      <tr>
        <td id="L163" class="blob-num js-line-number" data-line-number="163"></td>
        <td id="LC163" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//	if(vsp &lt; 6) </span></td>
      </tr>
      <tr>
        <td id="L164" class="blob-num js-line-number" data-line-number="164"></td>
        <td id="LC164" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//		return VSP_LT6;</span></td>
      </tr>
      <tr>
        <td id="L165" class="blob-num js-line-number" data-line-number="165"></td>
        <td id="LC165" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//	if(vsp &lt; 12) </span></td>
      </tr>
      <tr>
        <td id="L166" class="blob-num js-line-number" data-line-number="166"></td>
        <td id="LC166" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//		return VSP_6_12;</span></td>
      </tr>
      <tr>
        <td id="L167" class="blob-num js-line-number" data-line-number="167"></td>
        <td id="LC167" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//	if (vsp &lt; 18)</span></td>
      </tr>
      <tr>
        <td id="L168" class="blob-num js-line-number" data-line-number="168"></td>
        <td id="LC168" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//		return VSP_12_18;</span></td>
      </tr>
      <tr>
        <td id="L169" class="blob-num js-line-number" data-line-number="169"></td>
        <td id="LC169" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//	if (vsp &lt; 24)</span></td>
      </tr>
      <tr>
        <td id="L170" class="blob-num js-line-number" data-line-number="170"></td>
        <td id="LC170" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//		return VSP_18_24;</span></td>
      </tr>
      <tr>
        <td id="L171" class="blob-num js-line-number" data-line-number="171"></td>
        <td id="LC171" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//	if (vsp &lt; 30)</span></td>
      </tr>
      <tr>
        <td id="L172" class="blob-num js-line-number" data-line-number="172"></td>
        <td id="LC172" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//		return VSP_24_30;</span></td>
      </tr>
      <tr>
        <td id="L173" class="blob-num js-line-number" data-line-number="173"></td>
        <td id="LC173" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//	else</span></td>
      </tr>
      <tr>
        <td id="L174" class="blob-num js-line-number" data-line-number="174"></td>
        <td id="LC174" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//		return VSP_GT30;</span></td>
      </tr>
      <tr>
        <td id="L175" class="blob-num js-line-number" data-line-number="175"></td>
        <td id="LC175" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//}</span></td>
      </tr>
      <tr>
        <td id="L176" class="blob-num js-line-number" data-line-number="176"></td>
        <td id="LC176" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L177" class="blob-num js-line-number" data-line-number="177"></td>
        <td id="LC177" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L178" class="blob-num js-line-number" data-line-number="178"></td>
        <td id="LC178" class="blob-code blob-code-inner js-file-line"><span class="pl-k">void</span> <span class="pl-en">ReadInputEmissionRateFile</span>()</td>
      </tr>
      <tr>
        <td id="L179" class="blob-num js-line-number" data-line-number="179"></td>
        <td id="LC179" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L180" class="blob-num js-line-number" data-line-number="180"></td>
        <td id="LC180" class="blob-code blob-code-inner js-file-line">	CCSVParser parser_emission;</td>
      </tr>
      <tr>
        <td id="L181" class="blob-num js-line-number" data-line-number="181"></td>
        <td id="LC181" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L182" class="blob-num js-line-number" data-line-number="182"></td>
        <td id="LC182" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> line=<span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L183" class="blob-num js-line-number" data-line-number="183"></td>
        <td id="LC183" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (parser_emission.<span class="pl-c1">OpenCSVFile</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>input_vehicle_emission_rate.csv<span class="pl-pds">&quot;</span></span>))</td>
      </tr>
      <tr>
        <td id="L184" class="blob-num js-line-number" data-line-number="184"></td>
        <td id="LC184" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L185" class="blob-num js-line-number" data-line-number="185"></td>
        <td id="LC185" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L186" class="blob-num js-line-number" data-line-number="186"></td>
        <td id="LC186" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">while</span>(parser_emission.<span class="pl-c1">ReadRecord</span>())</td>
      </tr>
      <tr>
        <td id="L187" class="blob-num js-line-number" data-line-number="187"></td>
        <td id="LC187" class="blob-code blob-code-inner js-file-line">		{ line++;</td>
      </tr>
      <tr>
        <td id="L188" class="blob-num js-line-number" data-line-number="188"></td>
        <td id="LC188" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L189" class="blob-num js-line-number" data-line-number="189"></td>
        <td id="LC189" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">if</span>(line == <span class="pl-c1">256</span>)</td>
      </tr>
      <tr>
        <td id="L190" class="blob-num js-line-number" data-line-number="190"></td>
        <td id="LC190" class="blob-code blob-code-inner js-file-line">			<span class="pl-c1">TRACE</span>(<span class="pl-s"><span class="pl-pds">&quot;</span><span class="pl-pds">&quot;</span></span>);</td>
      </tr>
      <tr>
        <td id="L191" class="blob-num js-line-number" data-line-number="191"></td>
        <td id="LC191" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L192" class="blob-num js-line-number" data-line-number="192"></td>
        <td id="LC192" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L193" class="blob-num js-line-number" data-line-number="193"></td>
        <td id="LC193" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">int</span> vehicle_type;</td>
      </tr>
      <tr>
        <td id="L194" class="blob-num js-line-number" data-line-number="194"></td>
        <td id="LC194" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">int</span> opModeID;</td>
      </tr>
      <tr>
        <td id="L195" class="blob-num js-line-number" data-line-number="195"></td>
        <td id="LC195" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L196" class="blob-num js-line-number" data-line-number="196"></td>
        <td id="LC196" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span>(parser_emission.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>vehicle_type<span class="pl-pds">&quot;</span></span>,vehicle_type) == <span class="pl-c1">false</span>)</td>
      </tr>
      <tr>
        <td id="L197" class="blob-num js-line-number" data-line-number="197"></td>
        <td id="LC197" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">break</span>;</td>
      </tr>
      <tr>
        <td id="L198" class="blob-num js-line-number" data-line-number="198"></td>
        <td id="LC198" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span>(parser_emission.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>OpModeID<span class="pl-pds">&quot;</span></span>,opModeID) == <span class="pl-c1">false</span>)</td>
      </tr>
      <tr>
        <td id="L199" class="blob-num js-line-number" data-line-number="199"></td>
        <td id="LC199" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">break</span>;</td>
      </tr>
      <tr>
        <td id="L200" class="blob-num js-line-number" data-line-number="200"></td>
        <td id="LC200" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L201" class="blob-num js-line-number" data-line-number="201"></td>
        <td id="LC201" class="blob-code blob-code-inner js-file-line">			CEmissionRate element;</td>
      </tr>
      <tr>
        <td id="L202" class="blob-num js-line-number" data-line-number="202"></td>
        <td id="LC202" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span>(parser_emission.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>meanBaseRate_TotalEnergy_(KJ/hr)<span class="pl-pds">&quot;</span></span>,element.<span class="pl-smi">meanBaseRate_TotalEnergy</span>) == <span class="pl-c1">false</span>)</td>
      </tr>
      <tr>
        <td id="L203" class="blob-num js-line-number" data-line-number="203"></td>
        <td id="LC203" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">break</span>;</td>
      </tr>
      <tr>
        <td id="L204" class="blob-num js-line-number" data-line-number="204"></td>
        <td id="LC204" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span>(parser_emission.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>meanBaseRate_CO2_(g/hr)<span class="pl-pds">&quot;</span></span>,element.<span class="pl-smi">meanBaseRate_CO2</span>) == <span class="pl-c1">false</span>)</td>
      </tr>
      <tr>
        <td id="L205" class="blob-num js-line-number" data-line-number="205"></td>
        <td id="LC205" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">break</span>;</td>
      </tr>
      <tr>
        <td id="L206" class="blob-num js-line-number" data-line-number="206"></td>
        <td id="LC206" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span>(parser_emission.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>meanBaseRate_NOX_(g/hr)<span class="pl-pds">&quot;</span></span>,element.<span class="pl-smi">meanBaseRate_NOX</span>) == <span class="pl-c1">false</span>)</td>
      </tr>
      <tr>
        <td id="L207" class="blob-num js-line-number" data-line-number="207"></td>
        <td id="LC207" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">break</span>;</td>
      </tr>
      <tr>
        <td id="L208" class="blob-num js-line-number" data-line-number="208"></td>
        <td id="LC208" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span>(parser_emission.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>meanBaseRate_CO_(g/hr)<span class="pl-pds">&quot;</span></span>,element.<span class="pl-smi">meanBaseRate_CO</span>) == <span class="pl-c1">false</span>)</td>
      </tr>
      <tr>
        <td id="L209" class="blob-num js-line-number" data-line-number="209"></td>
        <td id="LC209" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">break</span>;</td>
      </tr>
      <tr>
        <td id="L210" class="blob-num js-line-number" data-line-number="210"></td>
        <td id="LC210" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span>(parser_emission.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>meanBaseRate_HC_(g/hr)<span class="pl-pds">&quot;</span></span>,element.<span class="pl-smi">meanBaseRate_HC</span>) == <span class="pl-c1">false</span>)</td>
      </tr>
      <tr>
        <td id="L211" class="blob-num js-line-number" data-line-number="211"></td>
        <td id="LC211" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">break</span>;</td>
      </tr>
      <tr>
        <td id="L212" class="blob-num js-line-number" data-line-number="212"></td>
        <td id="LC212" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L213" class="blob-num js-line-number" data-line-number="213"></td>
        <td id="LC213" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L214" class="blob-num js-line-number" data-line-number="214"></td>
        <td id="LC214" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span>(parser_emission.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>age<span class="pl-pds">&quot;</span></span>,element.<span class="pl-smi">Age</span>) == <span class="pl-c1">false</span>)</td>
      </tr>
      <tr>
        <td id="L215" class="blob-num js-line-number" data-line-number="215"></td>
        <td id="LC215" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L216" class="blob-num js-line-number" data-line-number="216"></td>
        <td id="LC216" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">break</span>;</td>
      </tr>
      <tr>
        <td id="L217" class="blob-num js-line-number" data-line-number="217"></td>
        <td id="LC217" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L218" class="blob-num js-line-number" data-line-number="218"></td>
        <td id="LC218" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L219" class="blob-num js-line-number" data-line-number="219"></td>
        <td id="LC219" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L220" class="blob-num js-line-number" data-line-number="220"></td>
        <td id="LC220" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span>(element.<span class="pl-smi">Age</span>&lt; _MAXIMUM_AGE_SIZE &amp;&amp; opModeID &lt; _MAXIMUM_OPERATING_MODE_SIZE)</td>
      </tr>
      <tr>
        <td id="L221" class="blob-num js-line-number" data-line-number="221"></td>
        <td id="LC221" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L222" class="blob-num js-line-number" data-line-number="222"></td>
        <td id="LC222" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L223" class="blob-num js-line-number" data-line-number="223"></td>
        <td id="LC223" class="blob-code blob-code-inner js-file-line">			EmissionRateData[vehicle_type][opModeID][element.<span class="pl-smi">Age</span>] = element;</td>
      </tr>
      <tr>
        <td id="L224" class="blob-num js-line-number" data-line-number="224"></td>
        <td id="LC224" class="blob-code blob-code-inner js-file-line">			EmissionRateData[vehicle_type][opModeID][element.<span class="pl-smi">Age</span>].<span class="pl-smi">bInitialized</span> = <span class="pl-c1">true</span>;</td>
      </tr>
      <tr>
        <td id="L225" class="blob-num js-line-number" data-line-number="225"></td>
        <td id="LC225" class="blob-code blob-code-inner js-file-line">			}<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L226" class="blob-num js-line-number" data-line-number="226"></td>
        <td id="LC226" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L227" class="blob-num js-line-number" data-line-number="227"></td>
        <td id="LC227" class="blob-code blob-code-inner js-file-line">			cout &lt;&lt; <span class="pl-s"><span class="pl-pds">&quot;</span>Reading error at line <span class="pl-pds">&quot;</span></span>  &lt;&lt; line &lt;&lt;<span class="pl-s"><span class="pl-pds">&quot;</span> at input_vehicle_emission_rate.csv<span class="pl-pds">&quot;</span></span> &lt;&lt;endl;</td>
      </tr>
      <tr>
        <td id="L228" class="blob-num js-line-number" data-line-number="228"></td>
        <td id="LC228" class="blob-code blob-code-inner js-file-line">			<span class="pl-c1">g_ProgramStop</span>();</td>
      </tr>
      <tr>
        <td id="L229" class="blob-num js-line-number" data-line-number="229"></td>
        <td id="LC229" class="blob-code blob-code-inner js-file-line">			</td>
      </tr>
      <tr>
        <td id="L230" class="blob-num js-line-number" data-line-number="230"></td>
        <td id="LC230" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L231" class="blob-num js-line-number" data-line-number="231"></td>
        <td id="LC231" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L232" class="blob-num js-line-number" data-line-number="232"></td>
        <td id="LC232" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L233" class="blob-num js-line-number" data-line-number="233"></td>
        <td id="LC233" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L234" class="blob-num js-line-number" data-line-number="234"></td>
        <td id="LC234" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L235" class="blob-num js-line-number" data-line-number="235"></td>
        <td id="LC235" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L236" class="blob-num js-line-number" data-line-number="236"></td>
        <td id="LC236" class="blob-code blob-code-inner js-file-line">		cout &lt;&lt; <span class="pl-s"><span class="pl-pds">&quot;</span>Error: File input_vehicle_emission_rate.csv cannot be opened.<span class="pl-cce">\n</span> It might be currently used and locked by EXCEL.<span class="pl-pds">&quot;</span></span>&lt;&lt; endl;</td>
      </tr>
      <tr>
        <td id="L237" class="blob-num js-line-number" data-line-number="237"></td>
        <td id="LC237" class="blob-code blob-code-inner js-file-line">		<span class="pl-c1">g_ProgramStop</span>();</td>
      </tr>
      <tr>
        <td id="L238" class="blob-num js-line-number" data-line-number="238"></td>
        <td id="LC238" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L239" class="blob-num js-line-number" data-line-number="239"></td>
        <td id="LC239" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L240" class="blob-num js-line-number" data-line-number="240"></td>
        <td id="LC240" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L241" class="blob-num js-line-number" data-line-number="241"></td>
        <td id="LC241" class="blob-code blob-code-inner js-file-line">	cout &lt;&lt; <span class="pl-s"><span class="pl-pds">&quot;</span>Read <span class="pl-pds">&quot;</span></span> &lt;&lt; line &lt;&lt; <span class="pl-s"><span class="pl-pds">&quot;</span> lines from file input_vehicle_emission_rate.csv.<span class="pl-pds">&quot;</span></span>&lt;&lt; endl;</td>
      </tr>
      <tr>
        <td id="L242" class="blob-num js-line-number" data-line-number="242"></td>
        <td id="LC242" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L243" class="blob-num js-line-number" data-line-number="243"></td>
        <td id="LC243" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L244" class="blob-num js-line-number" data-line-number="244"></td>
        <td id="LC244" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//typedef struct</span></td>
      </tr>
      <tr>
        <td id="L245" class="blob-num js-line-number" data-line-number="245"></td>
        <td id="LC245" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//{</span></td>
      </tr>
      <tr>
        <td id="L246" class="blob-num js-line-number" data-line-number="246"></td>
        <td id="LC246" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	int second;</span></td>
      </tr>
      <tr>
        <td id="L247" class="blob-num js-line-number" data-line-number="247"></td>
        <td id="LC247" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	float speed;</span></td>
      </tr>
      <tr>
        <td id="L248" class="blob-num js-line-number" data-line-number="248"></td>
        <td id="LC248" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	float position;</span></td>
      </tr>
      <tr>
        <td id="L249" class="blob-num js-line-number" data-line-number="249"></td>
        <td id="LC249" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//} SecondData;</span></td>
      </tr>
      <tr>
        <td id="L250" class="blob-num js-line-number" data-line-number="250"></td>
        <td id="LC250" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L251" class="blob-num js-line-number" data-line-number="251"></td>
        <td id="LC251" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//class DrivingCycleSample</span></td>
      </tr>
      <tr>
        <td id="L252" class="blob-num js-line-number" data-line-number="252"></td>
        <td id="LC252" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//{</span></td>
      </tr>
      <tr>
        <td id="L253" class="blob-num js-line-number" data-line-number="253"></td>
        <td id="LC253" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//public:</span></td>
      </tr>
      <tr>
        <td id="L254" class="blob-num js-line-number" data-line-number="254"></td>
        <td id="LC254" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	int driving_cycle_id;</span></td>
      </tr>
      <tr>
        <td id="L255" class="blob-num js-line-number" data-line-number="255"></td>
        <td id="LC255" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	vector&lt;SecondData&gt; dataVector;</span></td>
      </tr>
      <tr>
        <td id="L256" class="blob-num js-line-number" data-line-number="256"></td>
        <td id="LC256" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//};</span></td>
      </tr>
      <tr>
        <td id="L257" class="blob-num js-line-number" data-line-number="257"></td>
        <td id="LC257" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	//string s1 = &quot;Cycle_PC_PT_LCT.csv&quot;;</span></td>
      </tr>
      <tr>
        <td id="L258" class="blob-num js-line-number" data-line-number="258"></td>
        <td id="LC258" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	//string s2 = &quot;CYCLE_LHT.csv&quot;;</span></td>
      </tr>
      <tr>
        <td id="L259" class="blob-num js-line-number" data-line-number="259"></td>
        <td id="LC259" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	//string s3 = &quot;Cycle_SST.csv&quot;;</span></td>
      </tr>
      <tr>
        <td id="L260" class="blob-num js-line-number" data-line-number="260"></td>
        <td id="LC260" class="blob-code blob-code-inner js-file-line"><span class="pl-c">////map&lt;int, </span></td>
      </tr>
      <tr>
        <td id="L261" class="blob-num js-line-number" data-line-number="261"></td>
        <td id="LC261" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//void GenerateDrivingCycleSamples(int vehicle_type, string drivingCycleFileName)</span></td>
      </tr>
      <tr>
        <td id="L262" class="blob-num js-line-number" data-line-number="262"></td>
        <td id="LC262" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//{</span></td>
      </tr>
      <tr>
        <td id="L263" class="blob-num js-line-number" data-line-number="263"></td>
        <td id="LC263" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	FILE* pFile;</span></td>
      </tr>
      <tr>
        <td id="L264" class="blob-num js-line-number" data-line-number="264"></td>
        <td id="LC264" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	fopen_s(&amp;pFile, drivingCycleFileName.c_str(),&quot;r&quot;);</span></td>
      </tr>
      <tr>
        <td id="L265" class="blob-num js-line-number" data-line-number="265"></td>
        <td id="LC265" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	vector&lt;DrivingCycleSample&gt; drivingCycleSamples;</span></td>
      </tr>
      <tr>
        <td id="L266" class="blob-num js-line-number" data-line-number="266"></td>
        <td id="LC266" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	if (pFile)</span></td>
      </tr>
      <tr>
        <td id="L267" class="blob-num js-line-number" data-line-number="267"></td>
        <td id="LC267" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	{</span></td>
      </tr>
      <tr>
        <td id="L268" class="blob-num js-line-number" data-line-number="268"></td>
        <td id="LC268" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		int curr_cycle_id = -1;</span></td>
      </tr>
      <tr>
        <td id="L269" class="blob-num js-line-number" data-line-number="269"></td>
        <td id="LC269" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		int cycle_id, second;</span></td>
      </tr>
      <tr>
        <td id="L270" class="blob-num js-line-number" data-line-number="270"></td>
        <td id="LC270" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		float speed;</span></td>
      </tr>
      <tr>
        <td id="L271" class="blob-num js-line-number" data-line-number="271"></td>
        <td id="LC271" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		DrivingCycleSample samples;</span></td>
      </tr>
      <tr>
        <td id="L272" class="blob-num js-line-number" data-line-number="272"></td>
        <td id="LC272" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		while (!feof(pFile))</span></td>
      </tr>
      <tr>
        <td id="L273" class="blob-num js-line-number" data-line-number="273"></td>
        <td id="LC273" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		{</span></td>
      </tr>
      <tr>
        <td id="L274" class="blob-num js-line-number" data-line-number="274"></td>
        <td id="LC274" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			fscanf(pFile,&quot;%d,%d,%f\n&quot;,&amp;cycle_id, &amp;second, &amp;speed);</span></td>
      </tr>
      <tr>
        <td id="L275" class="blob-num js-line-number" data-line-number="275"></td>
        <td id="LC275" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			if (cycle_id != curr_cycle_id)</span></td>
      </tr>
      <tr>
        <td id="L276" class="blob-num js-line-number" data-line-number="276"></td>
        <td id="LC276" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			{</span></td>
      </tr>
      <tr>
        <td id="L277" class="blob-num js-line-number" data-line-number="277"></td>
        <td id="LC277" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//				samples.driving_cycle_id = cycle_id;</span></td>
      </tr>
      <tr>
        <td id="L278" class="blob-num js-line-number" data-line-number="278"></td>
        <td id="LC278" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			}</span></td>
      </tr>
      <tr>
        <td id="L279" class="blob-num js-line-number" data-line-number="279"></td>
        <td id="LC279" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			SecondData data;</span></td>
      </tr>
      <tr>
        <td id="L280" class="blob-num js-line-number" data-line-number="280"></td>
        <td id="LC280" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			data.second = second;</span></td>
      </tr>
      <tr>
        <td id="L281" class="blob-num js-line-number" data-line-number="281"></td>
        <td id="LC281" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			data.speed = speed;</span></td>
      </tr>
      <tr>
        <td id="L282" class="blob-num js-line-number" data-line-number="282"></td>
        <td id="LC282" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			samples.dataVector.push_back();</span></td>
      </tr>
      <tr>
        <td id="L283" class="blob-num js-line-number" data-line-number="283"></td>
        <td id="LC283" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		}</span></td>
      </tr>
      <tr>
        <td id="L284" class="blob-num js-line-number" data-line-number="284"></td>
        <td id="LC284" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	}</span></td>
      </tr>
      <tr>
        <td id="L285" class="blob-num js-line-number" data-line-number="285"></td>
        <td id="LC285" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//}</span></td>
      </tr>
      <tr>
        <td id="L286" class="blob-num js-line-number" data-line-number="286"></td>
        <td id="LC286" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L287" class="blob-num js-line-number" data-line-number="287"></td>
        <td id="LC287" class="blob-code blob-code-inner js-file-line"><span class="pl-k">enum</span> Pollutants {EMISSION_CO2,EMISSION_NOX,EMISSION_CO,EMISSION_HC,MAX_POLLUTANT};</td>
      </tr>
      <tr>
        <td id="L288" class="blob-num js-line-number" data-line-number="288"></td>
        <td id="LC288" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L289" class="blob-num js-line-number" data-line-number="289"></td>
        <td id="LC289" class="blob-code blob-code-inner js-file-line"><span class="pl-k">float</span> BaseCycleFractionOfOperatingMode[MAX_VEHICLE_TYPE_SIZE][<span class="pl-c1">41</span>] = {<span class="pl-c1">0</span>.<span class="pl-c1">0f</span>};</td>
      </tr>
      <tr>
        <td id="L290" class="blob-num js-line-number" data-line-number="290"></td>
        <td id="LC290" class="blob-code blob-code-inner js-file-line"><span class="pl-k">float</span> BaseCycleEmissionRate[MAX_VEHICLE_TYPE_SIZE][_MAXIMUM_AGE_SIZE][MAX_POLLUTANT] = {<span class="pl-c1">0</span>.<span class="pl-c1">0f</span>};</td>
      </tr>
      <tr>
        <td id="L291" class="blob-num js-line-number" data-line-number="291"></td>
        <td id="LC291" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L292" class="blob-num js-line-number" data-line-number="292"></td>
        <td id="LC292" class="blob-code blob-code-inner js-file-line"><span class="pl-k">void</span> <span class="pl-en">ReadFractionOfOperatingModeForBaseCycle</span>()</td>
      </tr>
      <tr>
        <td id="L293" class="blob-num js-line-number" data-line-number="293"></td>
        <td id="LC293" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L294" class="blob-num js-line-number" data-line-number="294"></td>
        <td id="LC294" class="blob-code blob-code-inner js-file-line">	CCSVParser parser_emission_factor;</td>
      </tr>
      <tr>
        <td id="L295" class="blob-num js-line-number" data-line-number="295"></td>
        <td id="LC295" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">OpenCSVFile</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>input_base_cycle_fraction_of_OpMode.csv<span class="pl-pds">&quot;</span></span>))</td>
      </tr>
      <tr>
        <td id="L296" class="blob-num js-line-number" data-line-number="296"></td>
        <td id="LC296" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L297" class="blob-num js-line-number" data-line-number="297"></td>
        <td id="LC297" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">int</span> vehicle_type;</td>
      </tr>
      <tr>
        <td id="L298" class="blob-num js-line-number" data-line-number="298"></td>
        <td id="LC298" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">float</span> value;</td>
      </tr>
      <tr>
        <td id="L299" class="blob-num js-line-number" data-line-number="299"></td>
        <td id="LC299" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L300" class="blob-num js-line-number" data-line-number="300"></td>
        <td id="LC300" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">while</span>(parser_emission_factor.<span class="pl-c1">ReadRecord</span>())</td>
      </tr>
      <tr>
        <td id="L301" class="blob-num js-line-number" data-line-number="301"></td>
        <td id="LC301" class="blob-code blob-code-inner js-file-line">		{			</td>
      </tr>
      <tr>
        <td id="L302" class="blob-num js-line-number" data-line-number="302"></td>
        <td id="LC302" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>vehicle_type<span class="pl-pds">&quot;</span></span>,vehicle_type) == <span class="pl-c1">false</span>)</td>
      </tr>
      <tr>
        <td id="L303" class="blob-num js-line-number" data-line-number="303"></td>
        <td id="LC303" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L304" class="blob-num js-line-number" data-line-number="304"></td>
        <td id="LC304" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">break</span>;</td>
      </tr>
      <tr>
        <td id="L305" class="blob-num js-line-number" data-line-number="305"></td>
        <td id="LC305" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L306" class="blob-num js-line-number" data-line-number="306"></td>
        <td id="LC306" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L307" class="blob-num js-line-number" data-line-number="307"></td>
        <td id="LC307" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L308" class="blob-num js-line-number" data-line-number="308"></td>
        <td id="LC308" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (vehicle_type &gt;= MAX_VEHICLE_TYPE_SIZE)</td>
      </tr>
      <tr>
        <td id="L309" class="blob-num js-line-number" data-line-number="309"></td>
        <td id="LC309" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L310" class="blob-num js-line-number" data-line-number="310"></td>
        <td id="LC310" class="blob-code blob-code-inner js-file-line">					cout &lt;&lt; <span class="pl-s"><span class="pl-pds">&quot;</span>Warning: unknown vehicle_type <span class="pl-pds">&quot;</span></span> &lt;&lt; vehicle_type &lt;&lt; <span class="pl-s"><span class="pl-pds">&quot;</span> !<span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>;</td>
      </tr>
      <tr>
        <td id="L311" class="blob-num js-line-number" data-line-number="311"></td>
        <td id="LC311" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">continue</span>;</td>
      </tr>
      <tr>
        <td id="L312" class="blob-num js-line-number" data-line-number="312"></td>
        <td id="LC312" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L313" class="blob-num js-line-number" data-line-number="313"></td>
        <td id="LC313" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L314" class="blob-num js-line-number" data-line-number="314"></td>
        <td id="LC314" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>0<span class="pl-pds">&quot;</span></span>, value))</td>
      </tr>
      <tr>
        <td id="L315" class="blob-num js-line-number" data-line-number="315"></td>
        <td id="LC315" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L316" class="blob-num js-line-number" data-line-number="316"></td>
        <td id="LC316" class="blob-code blob-code-inner js-file-line">					BaseCycleFractionOfOperatingMode[vehicle_type][<span class="pl-c1">0</span>] = value;</td>
      </tr>
      <tr>
        <td id="L317" class="blob-num js-line-number" data-line-number="317"></td>
        <td id="LC317" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L318" class="blob-num js-line-number" data-line-number="318"></td>
        <td id="LC318" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L319" class="blob-num js-line-number" data-line-number="319"></td>
        <td id="LC319" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>1<span class="pl-pds">&quot;</span></span>, value))</td>
      </tr>
      <tr>
        <td id="L320" class="blob-num js-line-number" data-line-number="320"></td>
        <td id="LC320" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L321" class="blob-num js-line-number" data-line-number="321"></td>
        <td id="LC321" class="blob-code blob-code-inner js-file-line">					BaseCycleFractionOfOperatingMode[vehicle_type][<span class="pl-c1">1</span>] = value;</td>
      </tr>
      <tr>
        <td id="L322" class="blob-num js-line-number" data-line-number="322"></td>
        <td id="LC322" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L323" class="blob-num js-line-number" data-line-number="323"></td>
        <td id="LC323" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L324" class="blob-num js-line-number" data-line-number="324"></td>
        <td id="LC324" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>11<span class="pl-pds">&quot;</span></span>, value))</td>
      </tr>
      <tr>
        <td id="L325" class="blob-num js-line-number" data-line-number="325"></td>
        <td id="LC325" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L326" class="blob-num js-line-number" data-line-number="326"></td>
        <td id="LC326" class="blob-code blob-code-inner js-file-line">					BaseCycleFractionOfOperatingMode[vehicle_type][<span class="pl-c1">11</span>] = value;</td>
      </tr>
      <tr>
        <td id="L327" class="blob-num js-line-number" data-line-number="327"></td>
        <td id="LC327" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L328" class="blob-num js-line-number" data-line-number="328"></td>
        <td id="LC328" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L329" class="blob-num js-line-number" data-line-number="329"></td>
        <td id="LC329" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>12<span class="pl-pds">&quot;</span></span>, value))</td>
      </tr>
      <tr>
        <td id="L330" class="blob-num js-line-number" data-line-number="330"></td>
        <td id="LC330" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L331" class="blob-num js-line-number" data-line-number="331"></td>
        <td id="LC331" class="blob-code blob-code-inner js-file-line">					BaseCycleFractionOfOperatingMode[vehicle_type][<span class="pl-c1">12</span>] = value;</td>
      </tr>
      <tr>
        <td id="L332" class="blob-num js-line-number" data-line-number="332"></td>
        <td id="LC332" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L333" class="blob-num js-line-number" data-line-number="333"></td>
        <td id="LC333" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L334" class="blob-num js-line-number" data-line-number="334"></td>
        <td id="LC334" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>13<span class="pl-pds">&quot;</span></span>, value))</td>
      </tr>
      <tr>
        <td id="L335" class="blob-num js-line-number" data-line-number="335"></td>
        <td id="LC335" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L336" class="blob-num js-line-number" data-line-number="336"></td>
        <td id="LC336" class="blob-code blob-code-inner js-file-line">					BaseCycleFractionOfOperatingMode[vehicle_type][<span class="pl-c1">13</span>] = value;</td>
      </tr>
      <tr>
        <td id="L337" class="blob-num js-line-number" data-line-number="337"></td>
        <td id="LC337" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L338" class="blob-num js-line-number" data-line-number="338"></td>
        <td id="LC338" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L339" class="blob-num js-line-number" data-line-number="339"></td>
        <td id="LC339" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>14<span class="pl-pds">&quot;</span></span>, value))</td>
      </tr>
      <tr>
        <td id="L340" class="blob-num js-line-number" data-line-number="340"></td>
        <td id="LC340" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L341" class="blob-num js-line-number" data-line-number="341"></td>
        <td id="LC341" class="blob-code blob-code-inner js-file-line">					BaseCycleFractionOfOperatingMode[vehicle_type][<span class="pl-c1">14</span>] = value;</td>
      </tr>
      <tr>
        <td id="L342" class="blob-num js-line-number" data-line-number="342"></td>
        <td id="LC342" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L343" class="blob-num js-line-number" data-line-number="343"></td>
        <td id="LC343" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L344" class="blob-num js-line-number" data-line-number="344"></td>
        <td id="LC344" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>15<span class="pl-pds">&quot;</span></span>, value))</td>
      </tr>
      <tr>
        <td id="L345" class="blob-num js-line-number" data-line-number="345"></td>
        <td id="LC345" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L346" class="blob-num js-line-number" data-line-number="346"></td>
        <td id="LC346" class="blob-code blob-code-inner js-file-line">					BaseCycleFractionOfOperatingMode[vehicle_type][<span class="pl-c1">15</span>] = value;</td>
      </tr>
      <tr>
        <td id="L347" class="blob-num js-line-number" data-line-number="347"></td>
        <td id="LC347" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L348" class="blob-num js-line-number" data-line-number="348"></td>
        <td id="LC348" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L349" class="blob-num js-line-number" data-line-number="349"></td>
        <td id="LC349" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>16<span class="pl-pds">&quot;</span></span>, value))</td>
      </tr>
      <tr>
        <td id="L350" class="blob-num js-line-number" data-line-number="350"></td>
        <td id="LC350" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L351" class="blob-num js-line-number" data-line-number="351"></td>
        <td id="LC351" class="blob-code blob-code-inner js-file-line">					BaseCycleFractionOfOperatingMode[vehicle_type][<span class="pl-c1">16</span>] = value;</td>
      </tr>
      <tr>
        <td id="L352" class="blob-num js-line-number" data-line-number="352"></td>
        <td id="LC352" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L353" class="blob-num js-line-number" data-line-number="353"></td>
        <td id="LC353" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L354" class="blob-num js-line-number" data-line-number="354"></td>
        <td id="LC354" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>21<span class="pl-pds">&quot;</span></span>, value))</td>
      </tr>
      <tr>
        <td id="L355" class="blob-num js-line-number" data-line-number="355"></td>
        <td id="LC355" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L356" class="blob-num js-line-number" data-line-number="356"></td>
        <td id="LC356" class="blob-code blob-code-inner js-file-line">					BaseCycleFractionOfOperatingMode[vehicle_type][<span class="pl-c1">21</span>] = value;</td>
      </tr>
      <tr>
        <td id="L357" class="blob-num js-line-number" data-line-number="357"></td>
        <td id="LC357" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L358" class="blob-num js-line-number" data-line-number="358"></td>
        <td id="LC358" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L359" class="blob-num js-line-number" data-line-number="359"></td>
        <td id="LC359" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>22<span class="pl-pds">&quot;</span></span>, value))</td>
      </tr>
      <tr>
        <td id="L360" class="blob-num js-line-number" data-line-number="360"></td>
        <td id="LC360" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L361" class="blob-num js-line-number" data-line-number="361"></td>
        <td id="LC361" class="blob-code blob-code-inner js-file-line">					BaseCycleFractionOfOperatingMode[vehicle_type][<span class="pl-c1">22</span>] = value;</td>
      </tr>
      <tr>
        <td id="L362" class="blob-num js-line-number" data-line-number="362"></td>
        <td id="LC362" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L363" class="blob-num js-line-number" data-line-number="363"></td>
        <td id="LC363" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L364" class="blob-num js-line-number" data-line-number="364"></td>
        <td id="LC364" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>23<span class="pl-pds">&quot;</span></span>, value))</td>
      </tr>
      <tr>
        <td id="L365" class="blob-num js-line-number" data-line-number="365"></td>
        <td id="LC365" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L366" class="blob-num js-line-number" data-line-number="366"></td>
        <td id="LC366" class="blob-code blob-code-inner js-file-line">					BaseCycleFractionOfOperatingMode[vehicle_type][<span class="pl-c1">23</span>] = value;</td>
      </tr>
      <tr>
        <td id="L367" class="blob-num js-line-number" data-line-number="367"></td>
        <td id="LC367" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L368" class="blob-num js-line-number" data-line-number="368"></td>
        <td id="LC368" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L369" class="blob-num js-line-number" data-line-number="369"></td>
        <td id="LC369" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>24<span class="pl-pds">&quot;</span></span>, value))</td>
      </tr>
      <tr>
        <td id="L370" class="blob-num js-line-number" data-line-number="370"></td>
        <td id="LC370" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L371" class="blob-num js-line-number" data-line-number="371"></td>
        <td id="LC371" class="blob-code blob-code-inner js-file-line">					BaseCycleFractionOfOperatingMode[vehicle_type][<span class="pl-c1">24</span>] = value;</td>
      </tr>
      <tr>
        <td id="L372" class="blob-num js-line-number" data-line-number="372"></td>
        <td id="LC372" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L373" class="blob-num js-line-number" data-line-number="373"></td>
        <td id="LC373" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L374" class="blob-num js-line-number" data-line-number="374"></td>
        <td id="LC374" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>25<span class="pl-pds">&quot;</span></span>, value))</td>
      </tr>
      <tr>
        <td id="L375" class="blob-num js-line-number" data-line-number="375"></td>
        <td id="LC375" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L376" class="blob-num js-line-number" data-line-number="376"></td>
        <td id="LC376" class="blob-code blob-code-inner js-file-line">					BaseCycleFractionOfOperatingMode[vehicle_type][<span class="pl-c1">25</span>] = value;</td>
      </tr>
      <tr>
        <td id="L377" class="blob-num js-line-number" data-line-number="377"></td>
        <td id="LC377" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L378" class="blob-num js-line-number" data-line-number="378"></td>
        <td id="LC378" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L379" class="blob-num js-line-number" data-line-number="379"></td>
        <td id="LC379" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>27<span class="pl-pds">&quot;</span></span>, value))</td>
      </tr>
      <tr>
        <td id="L380" class="blob-num js-line-number" data-line-number="380"></td>
        <td id="LC380" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L381" class="blob-num js-line-number" data-line-number="381"></td>
        <td id="LC381" class="blob-code blob-code-inner js-file-line">					BaseCycleFractionOfOperatingMode[vehicle_type][<span class="pl-c1">27</span>] = value;</td>
      </tr>
      <tr>
        <td id="L382" class="blob-num js-line-number" data-line-number="382"></td>
        <td id="LC382" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L383" class="blob-num js-line-number" data-line-number="383"></td>
        <td id="LC383" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L384" class="blob-num js-line-number" data-line-number="384"></td>
        <td id="LC384" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>28<span class="pl-pds">&quot;</span></span>, value))</td>
      </tr>
      <tr>
        <td id="L385" class="blob-num js-line-number" data-line-number="385"></td>
        <td id="LC385" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L386" class="blob-num js-line-number" data-line-number="386"></td>
        <td id="LC386" class="blob-code blob-code-inner js-file-line">					BaseCycleFractionOfOperatingMode[vehicle_type][<span class="pl-c1">28</span>] = value;</td>
      </tr>
      <tr>
        <td id="L387" class="blob-num js-line-number" data-line-number="387"></td>
        <td id="LC387" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L388" class="blob-num js-line-number" data-line-number="388"></td>
        <td id="LC388" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L389" class="blob-num js-line-number" data-line-number="389"></td>
        <td id="LC389" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>29<span class="pl-pds">&quot;</span></span>, value))</td>
      </tr>
      <tr>
        <td id="L390" class="blob-num js-line-number" data-line-number="390"></td>
        <td id="LC390" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L391" class="blob-num js-line-number" data-line-number="391"></td>
        <td id="LC391" class="blob-code blob-code-inner js-file-line">					BaseCycleFractionOfOperatingMode[vehicle_type][<span class="pl-c1">29</span>] = value;</td>
      </tr>
      <tr>
        <td id="L392" class="blob-num js-line-number" data-line-number="392"></td>
        <td id="LC392" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L393" class="blob-num js-line-number" data-line-number="393"></td>
        <td id="LC393" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L394" class="blob-num js-line-number" data-line-number="394"></td>
        <td id="LC394" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>30<span class="pl-pds">&quot;</span></span>, value))</td>
      </tr>
      <tr>
        <td id="L395" class="blob-num js-line-number" data-line-number="395"></td>
        <td id="LC395" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L396" class="blob-num js-line-number" data-line-number="396"></td>
        <td id="LC396" class="blob-code blob-code-inner js-file-line">					BaseCycleFractionOfOperatingMode[vehicle_type][<span class="pl-c1">30</span>] = value;</td>
      </tr>
      <tr>
        <td id="L397" class="blob-num js-line-number" data-line-number="397"></td>
        <td id="LC397" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L398" class="blob-num js-line-number" data-line-number="398"></td>
        <td id="LC398" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L399" class="blob-num js-line-number" data-line-number="399"></td>
        <td id="LC399" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>33<span class="pl-pds">&quot;</span></span>, value))</td>
      </tr>
      <tr>
        <td id="L400" class="blob-num js-line-number" data-line-number="400"></td>
        <td id="LC400" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L401" class="blob-num js-line-number" data-line-number="401"></td>
        <td id="LC401" class="blob-code blob-code-inner js-file-line">					BaseCycleFractionOfOperatingMode[vehicle_type][<span class="pl-c1">33</span>] = value;</td>
      </tr>
      <tr>
        <td id="L402" class="blob-num js-line-number" data-line-number="402"></td>
        <td id="LC402" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L403" class="blob-num js-line-number" data-line-number="403"></td>
        <td id="LC403" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L404" class="blob-num js-line-number" data-line-number="404"></td>
        <td id="LC404" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>35<span class="pl-pds">&quot;</span></span>, value))</td>
      </tr>
      <tr>
        <td id="L405" class="blob-num js-line-number" data-line-number="405"></td>
        <td id="LC405" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L406" class="blob-num js-line-number" data-line-number="406"></td>
        <td id="LC406" class="blob-code blob-code-inner js-file-line">					BaseCycleFractionOfOperatingMode[vehicle_type][<span class="pl-c1">35</span>] = value;</td>
      </tr>
      <tr>
        <td id="L407" class="blob-num js-line-number" data-line-number="407"></td>
        <td id="LC407" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L408" class="blob-num js-line-number" data-line-number="408"></td>
        <td id="LC408" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L409" class="blob-num js-line-number" data-line-number="409"></td>
        <td id="LC409" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>37<span class="pl-pds">&quot;</span></span>, value))</td>
      </tr>
      <tr>
        <td id="L410" class="blob-num js-line-number" data-line-number="410"></td>
        <td id="LC410" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L411" class="blob-num js-line-number" data-line-number="411"></td>
        <td id="LC411" class="blob-code blob-code-inner js-file-line">					BaseCycleFractionOfOperatingMode[vehicle_type][<span class="pl-c1">37</span>] = value;</td>
      </tr>
      <tr>
        <td id="L412" class="blob-num js-line-number" data-line-number="412"></td>
        <td id="LC412" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L413" class="blob-num js-line-number" data-line-number="413"></td>
        <td id="LC413" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L414" class="blob-num js-line-number" data-line-number="414"></td>
        <td id="LC414" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>38<span class="pl-pds">&quot;</span></span>, value))</td>
      </tr>
      <tr>
        <td id="L415" class="blob-num js-line-number" data-line-number="415"></td>
        <td id="LC415" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L416" class="blob-num js-line-number" data-line-number="416"></td>
        <td id="LC416" class="blob-code blob-code-inner js-file-line">					BaseCycleFractionOfOperatingMode[vehicle_type][<span class="pl-c1">38</span>] = value;</td>
      </tr>
      <tr>
        <td id="L417" class="blob-num js-line-number" data-line-number="417"></td>
        <td id="LC417" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L418" class="blob-num js-line-number" data-line-number="418"></td>
        <td id="LC418" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L419" class="blob-num js-line-number" data-line-number="419"></td>
        <td id="LC419" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>39<span class="pl-pds">&quot;</span></span>, value))</td>
      </tr>
      <tr>
        <td id="L420" class="blob-num js-line-number" data-line-number="420"></td>
        <td id="LC420" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L421" class="blob-num js-line-number" data-line-number="421"></td>
        <td id="LC421" class="blob-code blob-code-inner js-file-line">					BaseCycleFractionOfOperatingMode[vehicle_type][<span class="pl-c1">39</span>] = value;</td>
      </tr>
      <tr>
        <td id="L422" class="blob-num js-line-number" data-line-number="422"></td>
        <td id="LC422" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L423" class="blob-num js-line-number" data-line-number="423"></td>
        <td id="LC423" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L424" class="blob-num js-line-number" data-line-number="424"></td>
        <td id="LC424" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>40<span class="pl-pds">&quot;</span></span>, value))</td>
      </tr>
      <tr>
        <td id="L425" class="blob-num js-line-number" data-line-number="425"></td>
        <td id="LC425" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L426" class="blob-num js-line-number" data-line-number="426"></td>
        <td id="LC426" class="blob-code blob-code-inner js-file-line">					BaseCycleFractionOfOperatingMode[vehicle_type][<span class="pl-c1">40</span>] = value;</td>
      </tr>
      <tr>
        <td id="L427" class="blob-num js-line-number" data-line-number="427"></td>
        <td id="LC427" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L428" class="blob-num js-line-number" data-line-number="428"></td>
        <td id="LC428" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L429" class="blob-num js-line-number" data-line-number="429"></td>
        <td id="LC429" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L430" class="blob-num js-line-number" data-line-number="430"></td>
        <td id="LC430" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L431" class="blob-num js-line-number" data-line-number="431"></td>
        <td id="LC431" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L432" class="blob-num js-line-number" data-line-number="432"></td>
        <td id="LC432" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L433" class="blob-num js-line-number" data-line-number="433"></td>
        <td id="LC433" class="blob-code blob-code-inner js-file-line">		cout &lt;&lt; <span class="pl-s"><span class="pl-pds">&quot;</span>Error: File input_base_cycle_fraction_of_OpMode.csv cannot be opened.<span class="pl-cce">\n</span> It might be currently used and locked by EXCEL.<span class="pl-pds">&quot;</span></span>&lt;&lt; endl;</td>
      </tr>
      <tr>
        <td id="L434" class="blob-num js-line-number" data-line-number="434"></td>
        <td id="LC434" class="blob-code blob-code-inner js-file-line">		<span class="pl-c1">g_ProgramStop</span>();</td>
      </tr>
      <tr>
        <td id="L435" class="blob-num js-line-number" data-line-number="435"></td>
        <td id="LC435" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L436" class="blob-num js-line-number" data-line-number="436"></td>
        <td id="LC436" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L437" class="blob-num js-line-number" data-line-number="437"></td>
        <td id="LC437" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">for</span> (<span class="pl-k">int</span> vType = <span class="pl-c1">0</span>; vType &lt; MAX_VEHICLE_TYPE_SIZE; vType++)</td>
      </tr>
      <tr>
        <td id="L438" class="blob-num js-line-number" data-line-number="438"></td>
        <td id="LC438" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L439" class="blob-num js-line-number" data-line-number="439"></td>
        <td id="LC439" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">for</span> (<span class="pl-k">int</span> vAge = <span class="pl-c1">0</span>; vAge &lt; _MAXIMUM_AGE_SIZE; vAge++)</td>
      </tr>
      <tr>
        <td id="L440" class="blob-num js-line-number" data-line-number="440"></td>
        <td id="LC440" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L441" class="blob-num js-line-number" data-line-number="441"></td>
        <td id="LC441" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">for</span> (<span class="pl-k">int</span> opMode = <span class="pl-c1">0</span>; opMode &lt; <span class="pl-c1">41</span>;opMode++)</td>
      </tr>
      <tr>
        <td id="L442" class="blob-num js-line-number" data-line-number="442"></td>
        <td id="LC442" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L443" class="blob-num js-line-number" data-line-number="443"></td>
        <td id="LC443" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (EmissionRateData[vType][opMode][vAge].<span class="pl-smi">bInitialized</span>)</td>
      </tr>
      <tr>
        <td id="L444" class="blob-num js-line-number" data-line-number="444"></td>
        <td id="LC444" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L445" class="blob-num js-line-number" data-line-number="445"></td>
        <td id="LC445" class="blob-code blob-code-inner js-file-line">					BaseCycleEmissionRate[vType][vAge][EMISSION_CO2] += BaseCycleFractionOfOperatingMode[vType][opMode] * EmissionRateData[vType][opMode][vAge].<span class="pl-smi">meanBaseRate_CO2</span>;</td>
      </tr>
      <tr>
        <td id="L446" class="blob-num js-line-number" data-line-number="446"></td>
        <td id="LC446" class="blob-code blob-code-inner js-file-line">					BaseCycleEmissionRate[vType][vAge][EMISSION_NOX] += BaseCycleFractionOfOperatingMode[vType][opMode] * EmissionRateData[vType][opMode][vAge].<span class="pl-smi">meanBaseRate_NOX</span>;</td>
      </tr>
      <tr>
        <td id="L447" class="blob-num js-line-number" data-line-number="447"></td>
        <td id="LC447" class="blob-code blob-code-inner js-file-line">					BaseCycleEmissionRate[vType][vAge][EMISSION_CO] += BaseCycleFractionOfOperatingMode[vType][opMode] * EmissionRateData[vType][opMode][vAge].<span class="pl-smi">meanBaseRate_CO</span>;</td>
      </tr>
      <tr>
        <td id="L448" class="blob-num js-line-number" data-line-number="448"></td>
        <td id="LC448" class="blob-code blob-code-inner js-file-line">					BaseCycleEmissionRate[vType][vAge][EMISSION_HC] += BaseCycleFractionOfOperatingMode[vType][opMode] * EmissionRateData[vType][opMode][vAge].<span class="pl-smi">meanBaseRate_HC</span>;</td>
      </tr>
      <tr>
        <td id="L449" class="blob-num js-line-number" data-line-number="449"></td>
        <td id="LC449" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L450" class="blob-num js-line-number" data-line-number="450"></td>
        <td id="LC450" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L451" class="blob-num js-line-number" data-line-number="451"></td>
        <td id="LC451" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L452" class="blob-num js-line-number" data-line-number="452"></td>
        <td id="LC452" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L453" class="blob-num js-line-number" data-line-number="453"></td>
        <td id="LC453" class="blob-code blob-code-inner js-file-line">	cout &lt;&lt; <span class="pl-s"><span class="pl-pds">&quot;</span>Reading file input_base_cycle_fraction_of_OpMode.csv...<span class="pl-pds">&quot;</span></span>&lt;&lt; endl;</td>
      </tr>
      <tr>
        <td id="L454" class="blob-num js-line-number" data-line-number="454"></td>
        <td id="LC454" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L455" class="blob-num js-line-number" data-line-number="455"></td>
        <td id="LC455" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L456" class="blob-num js-line-number" data-line-number="456"></td>
        <td id="LC456" class="blob-code blob-code-inner js-file-line"><span class="pl-k">typedef</span> <span class="pl-k">struct</span></td>
      </tr>
      <tr>
        <td id="L457" class="blob-num js-line-number" data-line-number="457"></td>
        <td id="LC457" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L458" class="blob-num js-line-number" data-line-number="458"></td>
        <td id="LC458" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> second;</td>
      </tr>
      <tr>
        <td id="L459" class="blob-num js-line-number" data-line-number="459"></td>
        <td id="LC459" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> speed;</td>
      </tr>
      <tr>
        <td id="L460" class="blob-num js-line-number" data-line-number="460"></td>
        <td id="LC460" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> position;</td>
      </tr>
      <tr>
        <td id="L461" class="blob-num js-line-number" data-line-number="461"></td>
        <td id="LC461" class="blob-code blob-code-inner js-file-line">} SecondData;</td>
      </tr>
      <tr>
        <td id="L462" class="blob-num js-line-number" data-line-number="462"></td>
        <td id="LC462" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L463" class="blob-num js-line-number" data-line-number="463"></td>
        <td id="LC463" class="blob-code blob-code-inner js-file-line"><span class="pl-k">class</span> <span class="pl-en">DrivingCycleSample</span></td>
      </tr>
      <tr>
        <td id="L464" class="blob-num js-line-number" data-line-number="464"></td>
        <td id="LC464" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L465" class="blob-num js-line-number" data-line-number="465"></td>
        <td id="LC465" class="blob-code blob-code-inner js-file-line"><span class="pl-k">public:</span></td>
      </tr>
      <tr>
        <td id="L466" class="blob-num js-line-number" data-line-number="466"></td>
        <td id="LC466" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> driving_cycle_id;</td>
      </tr>
      <tr>
        <td id="L467" class="blob-num js-line-number" data-line-number="467"></td>
        <td id="LC467" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> defaultTimeSpan;</td>
      </tr>
      <tr>
        <td id="L468" class="blob-num js-line-number" data-line-number="468"></td>
        <td id="LC468" class="blob-code blob-code-inner js-file-line">	vector&lt;SecondData&gt; dataVector;</td>
      </tr>
      <tr>
        <td id="L469" class="blob-num js-line-number" data-line-number="469"></td>
        <td id="LC469" class="blob-code blob-code-inner js-file-line">	<span class="pl-en">DrivingCycleSample</span>(<span class="pl-k">int</span> cycle_id, <span class="pl-k">int</span> timeSpan = <span class="pl-c1">300</span>)</td>
      </tr>
      <tr>
        <td id="L470" class="blob-num js-line-number" data-line-number="470"></td>
        <td id="LC470" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L471" class="blob-num js-line-number" data-line-number="471"></td>
        <td id="LC471" class="blob-code blob-code-inner js-file-line">		driving_cycle_id = cycle_id;</td>
      </tr>
      <tr>
        <td id="L472" class="blob-num js-line-number" data-line-number="472"></td>
        <td id="LC472" class="blob-code blob-code-inner js-file-line">		defaultTimeSpan = timeSpan;</td>
      </tr>
      <tr>
        <td id="L473" class="blob-num js-line-number" data-line-number="473"></td>
        <td id="LC473" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L474" class="blob-num js-line-number" data-line-number="474"></td>
        <td id="LC474" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L475" class="blob-num js-line-number" data-line-number="475"></td>
        <td id="LC475" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">void</span> <span class="pl-en">SetSegmentTimeSpan</span>(<span class="pl-k">int</span> timeSpan)</td>
      </tr>
      <tr>
        <td id="L476" class="blob-num js-line-number" data-line-number="476"></td>
        <td id="LC476" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L477" class="blob-num js-line-number" data-line-number="477"></td>
        <td id="LC477" class="blob-code blob-code-inner js-file-line">		defaultTimeSpan = timeSpan;</td>
      </tr>
      <tr>
        <td id="L478" class="blob-num js-line-number" data-line-number="478"></td>
        <td id="LC478" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L479" class="blob-num js-line-number" data-line-number="479"></td>
        <td id="LC479" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L480" class="blob-num js-line-number" data-line-number="480"></td>
        <td id="LC480" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> <span class="pl-en">GetSpeedByTime</span>(<span class="pl-k">unsigned</span> <span class="pl-k">int</span> segmentID, <span class="pl-k">int</span> time)</td>
      </tr>
      <tr>
        <td id="L481" class="blob-num js-line-number" data-line-number="481"></td>
        <td id="LC481" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L482" class="blob-num js-line-number" data-line-number="482"></td>
        <td id="LC482" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">if</span> (segmentID * defaultTimeSpan + <span class="pl-c1">time</span> &lt; dataVector.<span class="pl-c1">size</span>())</td>
      </tr>
      <tr>
        <td id="L483" class="blob-num js-line-number" data-line-number="483"></td>
        <td id="LC483" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L484" class="blob-num js-line-number" data-line-number="484"></td>
        <td id="LC484" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">return</span> dataVector[segmentID * defaultTimeSpan + <span class="pl-c1">time</span>].<span class="pl-smi">speed</span>;</td>
      </tr>
      <tr>
        <td id="L485" class="blob-num js-line-number" data-line-number="485"></td>
        <td id="LC485" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L486" class="blob-num js-line-number" data-line-number="486"></td>
        <td id="LC486" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L487" class="blob-num js-line-number" data-line-number="487"></td>
        <td id="LC487" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L488" class="blob-num js-line-number" data-line-number="488"></td>
        <td id="LC488" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">return</span> <span class="pl-c1">0</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L489" class="blob-num js-line-number" data-line-number="489"></td>
        <td id="LC489" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L490" class="blob-num js-line-number" data-line-number="490"></td>
        <td id="LC490" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L491" class="blob-num js-line-number" data-line-number="491"></td>
        <td id="LC491" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L492" class="blob-num js-line-number" data-line-number="492"></td>
        <td id="LC492" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> <span class="pl-en">GetPositionByTime</span>(<span class="pl-k">unsigned</span> <span class="pl-k">int</span> segmentID, <span class="pl-k">int</span> time)</td>
      </tr>
      <tr>
        <td id="L493" class="blob-num js-line-number" data-line-number="493"></td>
        <td id="LC493" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L494" class="blob-num js-line-number" data-line-number="494"></td>
        <td id="LC494" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">if</span> (segmentID * defaultTimeSpan + <span class="pl-c1">time</span> &gt;= dataVector.<span class="pl-c1">size</span>())</td>
      </tr>
      <tr>
        <td id="L495" class="blob-num js-line-number" data-line-number="495"></td>
        <td id="LC495" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L496" class="blob-num js-line-number" data-line-number="496"></td>
        <td id="LC496" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">return</span> -<span class="pl-c1">1</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L497" class="blob-num js-line-number" data-line-number="497"></td>
        <td id="LC497" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L498" class="blob-num js-line-number" data-line-number="498"></td>
        <td id="LC498" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L499" class="blob-num js-line-number" data-line-number="499"></td>
        <td id="LC499" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">float</span> position_offset;</td>
      </tr>
      <tr>
        <td id="L500" class="blob-num js-line-number" data-line-number="500"></td>
        <td id="LC500" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">if</span> (segmentID == <span class="pl-c1">0</span>)</td>
      </tr>
      <tr>
        <td id="L501" class="blob-num js-line-number" data-line-number="501"></td>
        <td id="LC501" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L502" class="blob-num js-line-number" data-line-number="502"></td>
        <td id="LC502" class="blob-code blob-code-inner js-file-line">			position_offset = <span class="pl-c1">0</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L503" class="blob-num js-line-number" data-line-number="503"></td>
        <td id="LC503" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L504" class="blob-num js-line-number" data-line-number="504"></td>
        <td id="LC504" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L505" class="blob-num js-line-number" data-line-number="505"></td>
        <td id="LC505" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L506" class="blob-num js-line-number" data-line-number="506"></td>
        <td id="LC506" class="blob-code blob-code-inner js-file-line">			position_offset = dataVector[segmentID * defaultTimeSpan].<span class="pl-smi">position</span>;</td>
      </tr>
      <tr>
        <td id="L507" class="blob-num js-line-number" data-line-number="507"></td>
        <td id="LC507" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L508" class="blob-num js-line-number" data-line-number="508"></td>
        <td id="LC508" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L509" class="blob-num js-line-number" data-line-number="509"></td>
        <td id="LC509" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">return</span> <span class="pl-c1">max</span>(<span class="pl-c1">0</span>.<span class="pl-c1">0f</span>, dataVector[segmentID * defaultTimeSpan + <span class="pl-c1">time</span>].<span class="pl-smi">position</span> - position_offset);</td>
      </tr>
      <tr>
        <td id="L510" class="blob-num js-line-number" data-line-number="510"></td>
        <td id="LC510" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L511" class="blob-num js-line-number" data-line-number="511"></td>
        <td id="LC511" class="blob-code blob-code-inner js-file-line">};</td>
      </tr>
      <tr>
        <td id="L512" class="blob-num js-line-number" data-line-number="512"></td>
        <td id="LC512" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L513" class="blob-num js-line-number" data-line-number="513"></td>
        <td id="LC513" class="blob-code blob-code-inner js-file-line"><span class="pl-k">typedef</span> <span class="pl-k">struct</span></td>
      </tr>
      <tr>
        <td id="L514" class="blob-num js-line-number" data-line-number="514"></td>
        <td id="LC514" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L515" class="blob-num js-line-number" data-line-number="515"></td>
        <td id="LC515" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> cycle_id;</td>
      </tr>
      <tr>
        <td id="L516" class="blob-num js-line-number" data-line-number="516"></td>
        <td id="LC516" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> segment_id;</td>
      </tr>
      <tr>
        <td id="L517" class="blob-num js-line-number" data-line-number="517"></td>
        <td id="LC517" class="blob-code blob-code-inner js-file-line">} DrivingCycleIdentifier;</td>
      </tr>
      <tr>
        <td id="L518" class="blob-num js-line-number" data-line-number="518"></td>
        <td id="LC518" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L519" class="blob-num js-line-number" data-line-number="519"></td>
        <td id="LC519" class="blob-code blob-code-inner js-file-line"><span class="pl-k">class</span> <span class="pl-en">TrajectoryMatrix</span></td>
      </tr>
      <tr>
        <td id="L520" class="blob-num js-line-number" data-line-number="520"></td>
        <td id="LC520" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L521" class="blob-num js-line-number" data-line-number="521"></td>
        <td id="LC521" class="blob-code blob-code-inner js-file-line"><span class="pl-k">public:</span></td>
      </tr>
      <tr>
        <td id="L522" class="blob-num js-line-number" data-line-number="522"></td>
        <td id="LC522" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> timeWindowSize;</td>
      </tr>
      <tr>
        <td id="L523" class="blob-num js-line-number" data-line-number="523"></td>
        <td id="LC523" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> spaceWindowSize;</td>
      </tr>
      <tr>
        <td id="L524" class="blob-num js-line-number" data-line-number="524"></td>
        <td id="LC524" class="blob-code blob-code-inner js-file-line"><span class="pl-k">private:</span></td>
      </tr>
      <tr>
        <td id="L525" class="blob-num js-line-number" data-line-number="525"></td>
        <td id="LC525" class="blob-code blob-code-inner js-file-line">	vector&lt;DrivingCycleIdentifier&gt;** pMatrix;</td>
      </tr>
      <tr>
        <td id="L526" class="blob-num js-line-number" data-line-number="526"></td>
        <td id="LC526" class="blob-code blob-code-inner js-file-line"><span class="pl-k">public:</span></td>
      </tr>
      <tr>
        <td id="L527" class="blob-num js-line-number" data-line-number="527"></td>
        <td id="LC527" class="blob-code blob-code-inner js-file-line">	<span class="pl-en">TrajectoryMatrix</span>(<span class="pl-k">int</span> n = <span class="pl-c1">300</span>, <span class="pl-k">int</span> m = <span class="pl-c1">800</span>)</td>
      </tr>
      <tr>
        <td id="L528" class="blob-num js-line-number" data-line-number="528"></td>
        <td id="LC528" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L529" class="blob-num js-line-number" data-line-number="529"></td>
        <td id="LC529" class="blob-code blob-code-inner js-file-line">		timeWindowSize = n;</td>
      </tr>
      <tr>
        <td id="L530" class="blob-num js-line-number" data-line-number="530"></td>
        <td id="LC530" class="blob-code blob-code-inner js-file-line">		spaceWindowSize = m;</td>
      </tr>
      <tr>
        <td id="L531" class="blob-num js-line-number" data-line-number="531"></td>
        <td id="LC531" class="blob-code blob-code-inner js-file-line">		pMatrix = <span class="pl-k">new</span> vector&lt;DrivingCycleIdentifier&gt;*[n];</td>
      </tr>
      <tr>
        <td id="L532" class="blob-num js-line-number" data-line-number="532"></td>
        <td id="LC532" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">for</span> (<span class="pl-k">int</span> i=<span class="pl-c1">0</span>;i&lt;n;i++)</td>
      </tr>
      <tr>
        <td id="L533" class="blob-num js-line-number" data-line-number="533"></td>
        <td id="LC533" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L534" class="blob-num js-line-number" data-line-number="534"></td>
        <td id="LC534" class="blob-code blob-code-inner js-file-line">			pMatrix[i] = <span class="pl-k">new</span> vector&lt;DrivingCycleIdentifier&gt;[m];</td>
      </tr>
      <tr>
        <td id="L535" class="blob-num js-line-number" data-line-number="535"></td>
        <td id="LC535" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L536" class="blob-num js-line-number" data-line-number="536"></td>
        <td id="LC536" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L537" class="blob-num js-line-number" data-line-number="537"></td>
        <td id="LC537" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L538" class="blob-num js-line-number" data-line-number="538"></td>
        <td id="LC538" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">void</span> <span class="pl-en">AddToMatrix</span>(DrivingCycleSample&amp; sample, <span class="pl-k">unsigned</span> <span class="pl-k">int</span> segment_id)</td>
      </tr>
      <tr>
        <td id="L539" class="blob-num js-line-number" data-line-number="539"></td>
        <td id="LC539" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L540" class="blob-num js-line-number" data-line-number="540"></td>
        <td id="LC540" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">for</span> (<span class="pl-k">int</span> i=<span class="pl-c1">0</span>;i&lt;sample.<span class="pl-smi">defaultTimeSpan</span>;i++)</td>
      </tr>
      <tr>
        <td id="L541" class="blob-num js-line-number" data-line-number="541"></td>
        <td id="LC541" class="blob-code blob-code-inner js-file-line">		{		</td>
      </tr>
      <tr>
        <td id="L542" class="blob-num js-line-number" data-line-number="542"></td>
        <td id="LC542" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">int</span> posIdx = (<span class="pl-k">int</span>)sample.<span class="pl-c1">GetPositionByTime</span>(segment_id,i);</td>
      </tr>
      <tr>
        <td id="L543" class="blob-num js-line-number" data-line-number="543"></td>
        <td id="LC543" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (posIdx &lt; <span class="pl-c1">0</span>) <span class="pl-k">break</span>;</td>
      </tr>
      <tr>
        <td id="L544" class="blob-num js-line-number" data-line-number="544"></td>
        <td id="LC544" class="blob-code blob-code-inner js-file-line">			DrivingCycleIdentifier identifier;</td>
      </tr>
      <tr>
        <td id="L545" class="blob-num js-line-number" data-line-number="545"></td>
        <td id="LC545" class="blob-code blob-code-inner js-file-line">			identifier.<span class="pl-smi">cycle_id</span> = sample.<span class="pl-smi">driving_cycle_id</span>;</td>
      </tr>
      <tr>
        <td id="L546" class="blob-num js-line-number" data-line-number="546"></td>
        <td id="LC546" class="blob-code blob-code-inner js-file-line">			identifier.<span class="pl-smi">segment_id</span> = segment_id;</td>
      </tr>
      <tr>
        <td id="L547" class="blob-num js-line-number" data-line-number="547"></td>
        <td id="LC547" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (i &lt; timeWindowSize &amp;&amp; posIdx &lt; spaceWindowSize)</td>
      </tr>
      <tr>
        <td id="L548" class="blob-num js-line-number" data-line-number="548"></td>
        <td id="LC548" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L549" class="blob-num js-line-number" data-line-number="549"></td>
        <td id="LC549" class="blob-code blob-code-inner js-file-line">				pMatrix[i][posIdx].<span class="pl-c1">push_back</span>(identifier);</td>
      </tr>
      <tr>
        <td id="L550" class="blob-num js-line-number" data-line-number="550"></td>
        <td id="LC550" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L551" class="blob-num js-line-number" data-line-number="551"></td>
        <td id="LC551" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L552" class="blob-num js-line-number" data-line-number="552"></td>
        <td id="LC552" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L553" class="blob-num js-line-number" data-line-number="553"></td>
        <td id="LC553" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L554" class="blob-num js-line-number" data-line-number="554"></td>
        <td id="LC554" class="blob-code blob-code-inner js-file-line">	vector&lt;DrivingCycleIdentifier&gt;* <span class="pl-en">GetIdentifier</span>(<span class="pl-k">int</span> n, <span class="pl-k">int</span> m) <span class="pl-k">const</span></td>
      </tr>
      <tr>
        <td id="L555" class="blob-num js-line-number" data-line-number="555"></td>
        <td id="LC555" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L556" class="blob-num js-line-number" data-line-number="556"></td>
        <td id="LC556" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">if</span> (n &lt; timeWindowSize &amp;&amp; m &lt; spaceWindowSize)</td>
      </tr>
      <tr>
        <td id="L557" class="blob-num js-line-number" data-line-number="557"></td>
        <td id="LC557" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L558" class="blob-num js-line-number" data-line-number="558"></td>
        <td id="LC558" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">return</span> &amp;pMatrix[n][m];</td>
      </tr>
      <tr>
        <td id="L559" class="blob-num js-line-number" data-line-number="559"></td>
        <td id="LC559" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L560" class="blob-num js-line-number" data-line-number="560"></td>
        <td id="LC560" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L561" class="blob-num js-line-number" data-line-number="561"></td>
        <td id="LC561" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">return</span> <span class="pl-c1">NULL</span>;</td>
      </tr>
      <tr>
        <td id="L562" class="blob-num js-line-number" data-line-number="562"></td>
        <td id="LC562" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L563" class="blob-num js-line-number" data-line-number="563"></td>
        <td id="LC563" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L564" class="blob-num js-line-number" data-line-number="564"></td>
        <td id="LC564" class="blob-code blob-code-inner js-file-line">	<span class="pl-en">~TrajectoryMatrix</span>()</td>
      </tr>
      <tr>
        <td id="L565" class="blob-num js-line-number" data-line-number="565"></td>
        <td id="LC565" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L566" class="blob-num js-line-number" data-line-number="566"></td>
        <td id="LC566" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">for</span> (<span class="pl-k">int</span> i=<span class="pl-c1">0</span>;i&lt;timeWindowSize;i++)</td>
      </tr>
      <tr>
        <td id="L567" class="blob-num js-line-number" data-line-number="567"></td>
        <td id="LC567" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L568" class="blob-num js-line-number" data-line-number="568"></td>
        <td id="LC568" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">delete []</span> pMatrix[i];</td>
      </tr>
      <tr>
        <td id="L569" class="blob-num js-line-number" data-line-number="569"></td>
        <td id="LC569" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L570" class="blob-num js-line-number" data-line-number="570"></td>
        <td id="LC570" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L571" class="blob-num js-line-number" data-line-number="571"></td>
        <td id="LC571" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">delete []</span> pMatrix;</td>
      </tr>
      <tr>
        <td id="L572" class="blob-num js-line-number" data-line-number="572"></td>
        <td id="LC572" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L573" class="blob-num js-line-number" data-line-number="573"></td>
        <td id="LC573" class="blob-code blob-code-inner js-file-line">};</td>
      </tr>
      <tr>
        <td id="L574" class="blob-num js-line-number" data-line-number="574"></td>
        <td id="LC574" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L575" class="blob-num js-line-number" data-line-number="575"></td>
        <td id="LC575" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//TrajectoryMatrix trajectoryMatrixBin[MAX_VEHICLE_TYPE_SIZE][7];</span></td>
      </tr>
      <tr>
        <td id="L576" class="blob-num js-line-number" data-line-number="576"></td>
        <td id="LC576" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L577" class="blob-num js-line-number" data-line-number="577"></td>
        <td id="LC577" class="blob-code blob-code-inner js-file-line"><span class="pl-k">int</span> <span class="pl-en">GetDrivingCycleVehicleSpeedBin</span>(<span class="pl-k">float</span> speed)</td>
      </tr>
      <tr>
        <td id="L578" class="blob-num js-line-number" data-line-number="578"></td>
        <td id="LC578" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L579" class="blob-num js-line-number" data-line-number="579"></td>
        <td id="LC579" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (speed &lt; <span class="pl-c1">15</span>.<span class="pl-c1">0f</span>) <span class="pl-k">return</span> <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L580" class="blob-num js-line-number" data-line-number="580"></td>
        <td id="LC580" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (speed &lt; <span class="pl-c1">30</span>.<span class="pl-c1">0f</span>) <span class="pl-k">return</span> <span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L581" class="blob-num js-line-number" data-line-number="581"></td>
        <td id="LC581" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (speed &lt; <span class="pl-c1">40</span>.<span class="pl-c1">0f</span>) <span class="pl-k">return</span> <span class="pl-c1">2</span>;</td>
      </tr>
      <tr>
        <td id="L582" class="blob-num js-line-number" data-line-number="582"></td>
        <td id="LC582" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (speed &lt; <span class="pl-c1">50</span>.<span class="pl-c1">0f</span>) <span class="pl-k">return</span> <span class="pl-c1">3</span>;</td>
      </tr>
      <tr>
        <td id="L583" class="blob-num js-line-number" data-line-number="583"></td>
        <td id="LC583" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (speed &lt; <span class="pl-c1">60</span>.<span class="pl-c1">0f</span>) <span class="pl-k">return</span> <span class="pl-c1">4</span>;</td>
      </tr>
      <tr>
        <td id="L584" class="blob-num js-line-number" data-line-number="584"></td>
        <td id="LC584" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (speed &lt; <span class="pl-c1">70</span>.<span class="pl-c1">0f</span>) <span class="pl-k">return</span> <span class="pl-c1">5</span>;</td>
      </tr>
      <tr>
        <td id="L585" class="blob-num js-line-number" data-line-number="585"></td>
        <td id="LC585" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L586" class="blob-num js-line-number" data-line-number="586"></td>
        <td id="LC586" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">return</span> <span class="pl-c1">6</span>;</td>
      </tr>
      <tr>
        <td id="L587" class="blob-num js-line-number" data-line-number="587"></td>
        <td id="LC587" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L588" class="blob-num js-line-number" data-line-number="588"></td>
        <td id="LC588" class="blob-code blob-code-inner js-file-line">map&lt;<span class="pl-k">int</span>,vector&lt;DrivingCycleSample*&gt;&gt; drivingCycleMap;</td>
      </tr>
      <tr>
        <td id="L589" class="blob-num js-line-number" data-line-number="589"></td>
        <td id="LC589" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L590" class="blob-num js-line-number" data-line-number="590"></td>
        <td id="LC590" class="blob-code blob-code-inner js-file-line"><span class="pl-k">void</span> <span class="pl-en">ReadInputCycleAverageEmissionFactors</span>()</td>
      </tr>
      <tr>
        <td id="L591" class="blob-num js-line-number" data-line-number="591"></td>
        <td id="LC591" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L592" class="blob-num js-line-number" data-line-number="592"></td>
        <td id="LC592" class="blob-code blob-code-inner js-file-line">	CCSVParser parser_emission_factor;</td>
      </tr>
      <tr>
        <td id="L593" class="blob-num js-line-number" data-line-number="593"></td>
        <td id="LC593" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">OpenCSVFile</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>input_cycle_emission_factor.csv<span class="pl-pds">&quot;</span></span>))</td>
      </tr>
      <tr>
        <td id="L594" class="blob-num js-line-number" data-line-number="594"></td>
        <td id="LC594" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L595" class="blob-num js-line-number" data-line-number="595"></td>
        <td id="LC595" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">int</span> vehicle_type;</td>
      </tr>
      <tr>
        <td id="L596" class="blob-num js-line-number" data-line-number="596"></td>
        <td id="LC596" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">float</span> emissionFactor_CO2;</td>
      </tr>
      <tr>
        <td id="L597" class="blob-num js-line-number" data-line-number="597"></td>
        <td id="LC597" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">float</span> emissionFactor_NOX;</td>
      </tr>
      <tr>
        <td id="L598" class="blob-num js-line-number" data-line-number="598"></td>
        <td id="LC598" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">float</span> emissionFactor_CO;</td>
      </tr>
      <tr>
        <td id="L599" class="blob-num js-line-number" data-line-number="599"></td>
        <td id="LC599" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">float</span> emissionFactor_HC;</td>
      </tr>
      <tr>
        <td id="L600" class="blob-num js-line-number" data-line-number="600"></td>
        <td id="LC600" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">float</span> cycle_average_speed;</td>
      </tr>
      <tr>
        <td id="L601" class="blob-num js-line-number" data-line-number="601"></td>
        <td id="LC601" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">int</span> age;</td>
      </tr>
      <tr>
        <td id="L602" class="blob-num js-line-number" data-line-number="602"></td>
        <td id="LC602" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L603" class="blob-num js-line-number" data-line-number="603"></td>
        <td id="LC603" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">while</span> (parser_emission_factor.<span class="pl-c1">ReadRecord</span>())</td>
      </tr>
      <tr>
        <td id="L604" class="blob-num js-line-number" data-line-number="604"></td>
        <td id="LC604" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L605" class="blob-num js-line-number" data-line-number="605"></td>
        <td id="LC605" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>vehicle_type<span class="pl-pds">&quot;</span></span>, vehicle_type) == <span class="pl-c1">false</span></td>
      </tr>
      <tr>
        <td id="L606" class="blob-num js-line-number" data-line-number="606"></td>
        <td id="LC606" class="blob-code blob-code-inner js-file-line">				|| parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>CO2_emission_factor_(g/mi)<span class="pl-pds">&quot;</span></span>, emissionFactor_CO2) == <span class="pl-c1">false</span></td>
      </tr>
      <tr>
        <td id="L607" class="blob-num js-line-number" data-line-number="607"></td>
        <td id="LC607" class="blob-code blob-code-inner js-file-line">				|| parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>NOX_emission_factor_(g/mi)<span class="pl-pds">&quot;</span></span>, emissionFactor_NOX) == <span class="pl-c1">false</span></td>
      </tr>
      <tr>
        <td id="L608" class="blob-num js-line-number" data-line-number="608"></td>
        <td id="LC608" class="blob-code blob-code-inner js-file-line">				|| parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>CO_emission_factor_(g/mi)<span class="pl-pds">&quot;</span></span>, emissionFactor_CO) == <span class="pl-c1">false</span></td>
      </tr>
      <tr>
        <td id="L609" class="blob-num js-line-number" data-line-number="609"></td>
        <td id="LC609" class="blob-code blob-code-inner js-file-line">				|| parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>HC_emission_factor_(g/mi)<span class="pl-pds">&quot;</span></span>, emissionFactor_HC) == <span class="pl-c1">false</span></td>
      </tr>
      <tr>
        <td id="L610" class="blob-num js-line-number" data-line-number="610"></td>
        <td id="LC610" class="blob-code blob-code-inner js-file-line">				|| parser_emission_factor.<span class="pl-c1">GetValueByFieldName</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>age<span class="pl-pds">&quot;</span></span>, age) == <span class="pl-c1">false</span></td>
      </tr>
      <tr>
        <td id="L611" class="blob-num js-line-number" data-line-number="611"></td>
        <td id="LC611" class="blob-code blob-code-inner js-file-line">				)</td>
      </tr>
      <tr>
        <td id="L612" class="blob-num js-line-number" data-line-number="612"></td>
        <td id="LC612" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L613" class="blob-num js-line-number" data-line-number="613"></td>
        <td id="LC613" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">break</span>;</td>
      </tr>
      <tr>
        <td id="L614" class="blob-num js-line-number" data-line-number="614"></td>
        <td id="LC614" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L615" class="blob-num js-line-number" data-line-number="615"></td>
        <td id="LC615" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L616" class="blob-num js-line-number" data-line-number="616"></td>
        <td id="LC616" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L617" class="blob-num js-line-number" data-line-number="617"></td>
        <td id="LC617" class="blob-code blob-code-inner js-file-line">				CycleAverageEmissionFactorMatrix[vehicle_type][age] = <span class="pl-c1">CCycleAverageEmissionFactor</span>(emissionFactor_CO2, emissionFactor_NOX, emissionFactor_CO, emissionFactor_HC);</td>
      </tr>
      <tr>
        <td id="L618" class="blob-num js-line-number" data-line-number="618"></td>
        <td id="LC618" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L619" class="blob-num js-line-number" data-line-number="619"></td>
        <td id="LC619" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L620" class="blob-num js-line-number" data-line-number="620"></td>
        <td id="LC620" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L621" class="blob-num js-line-number" data-line-number="621"></td>
        <td id="LC621" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L622" class="blob-num js-line-number" data-line-number="622"></td>
        <td id="LC622" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L623" class="blob-num js-line-number" data-line-number="623"></td>
        <td id="LC623" class="blob-code blob-code-inner js-file-line">		cout &lt;&lt; <span class="pl-s"><span class="pl-pds">&quot;</span>Error: File input_cycle_emission_factor.csv cannot be opened.<span class="pl-cce">\n</span> It might be currently used and locked by EXCEL.<span class="pl-pds">&quot;</span></span> &lt;&lt; endl;</td>
      </tr>
      <tr>
        <td id="L624" class="blob-num js-line-number" data-line-number="624"></td>
        <td id="LC624" class="blob-code blob-code-inner js-file-line">		<span class="pl-c1">g_ProgramStop</span>();</td>
      </tr>
      <tr>
        <td id="L625" class="blob-num js-line-number" data-line-number="625"></td>
        <td id="LC625" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L626" class="blob-num js-line-number" data-line-number="626"></td>
        <td id="LC626" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L627" class="blob-num js-line-number" data-line-number="627"></td>
        <td id="LC627" class="blob-code blob-code-inner js-file-line">	cout &lt;&lt; <span class="pl-s"><span class="pl-pds">&quot;</span>Reading file input_cycle_emission_factor.csv...<span class="pl-pds">&quot;</span></span> &lt;&lt; endl;</td>
      </tr>
      <tr>
        <td id="L628" class="blob-num js-line-number" data-line-number="628"></td>
        <td id="LC628" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L629" class="blob-num js-line-number" data-line-number="629"></td>
        <td id="LC629" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L630" class="blob-num js-line-number" data-line-number="630"></td>
        <td id="LC630" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//void ReadDrivingCycleSamplesByVehicleType(int vehicle_type, char* fileName, vector&lt;DrivingCycleSample*&gt;&amp; drivingCycleSamples)</span></td>
      </tr>
      <tr>
        <td id="L631" class="blob-num js-line-number" data-line-number="631"></td>
        <td id="LC631" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//{</span></td>
      </tr>
      <tr>
        <td id="L632" class="blob-num js-line-number" data-line-number="632"></td>
        <td id="LC632" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	FILE* pFile;</span></td>
      </tr>
      <tr>
        <td id="L633" class="blob-num js-line-number" data-line-number="633"></td>
        <td id="LC633" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	fopen_s(&amp;pFile,fileName, &quot;r&quot;);</span></td>
      </tr>
      <tr>
        <td id="L634" class="blob-num js-line-number" data-line-number="634"></td>
        <td id="LC634" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	int curr_cycle_id = -1;</span></td>
      </tr>
      <tr>
        <td id="L635" class="blob-num js-line-number" data-line-number="635"></td>
        <td id="LC635" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	int cycle_id, second;</span></td>
      </tr>
      <tr>
        <td id="L636" class="blob-num js-line-number" data-line-number="636"></td>
        <td id="LC636" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	float speed, position;</span></td>
      </tr>
      <tr>
        <td id="L637" class="blob-num js-line-number" data-line-number="637"></td>
        <td id="LC637" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	DrivingCycleSample* pDrivingCycleSample;</span></td>
      </tr>
      <tr>
        <td id="L638" class="blob-num js-line-number" data-line-number="638"></td>
        <td id="LC638" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	while (!feof(pFile))</span></td>
      </tr>
      <tr>
        <td id="L639" class="blob-num js-line-number" data-line-number="639"></td>
        <td id="LC639" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	{</span></td>
      </tr>
      <tr>
        <td id="L640" class="blob-num js-line-number" data-line-number="640"></td>
        <td id="LC640" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		fscanf_s(pFile,&quot;%d,%d,%f\n&quot;,&amp;cycle_id, &amp;second, &amp;speed);</span></td>
      </tr>
      <tr>
        <td id="L641" class="blob-num js-line-number" data-line-number="641"></td>
        <td id="LC641" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		if (curr_cycle_id != cycle_id)</span></td>
      </tr>
      <tr>
        <td id="L642" class="blob-num js-line-number" data-line-number="642"></td>
        <td id="LC642" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		{</span></td>
      </tr>
      <tr>
        <td id="L643" class="blob-num js-line-number" data-line-number="643"></td>
        <td id="LC643" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			curr_cycle_id = cycle_id;</span></td>
      </tr>
      <tr>
        <td id="L644" class="blob-num js-line-number" data-line-number="644"></td>
        <td id="LC644" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			position = 0.0f;</span></td>
      </tr>
      <tr>
        <td id="L645" class="blob-num js-line-number" data-line-number="645"></td>
        <td id="LC645" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			pDrivingCycleSample = new DrivingCycleSample(cycle_id);</span></td>
      </tr>
      <tr>
        <td id="L646" class="blob-num js-line-number" data-line-number="646"></td>
        <td id="LC646" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			drivingCycleSamples.push_back(pDrivingCycleSample);</span></td>
      </tr>
      <tr>
        <td id="L647" class="blob-num js-line-number" data-line-number="647"></td>
        <td id="LC647" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		}</span></td>
      </tr>
      <tr>
        <td id="L648" class="blob-num js-line-number" data-line-number="648"></td>
        <td id="LC648" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L649" class="blob-num js-line-number" data-line-number="649"></td>
        <td id="LC649" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		SecondData data;</span></td>
      </tr>
      <tr>
        <td id="L650" class="blob-num js-line-number" data-line-number="650"></td>
        <td id="LC650" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		data.second = second;</span></td>
      </tr>
      <tr>
        <td id="L651" class="blob-num js-line-number" data-line-number="651"></td>
        <td id="LC651" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		data.speed = speed;</span></td>
      </tr>
      <tr>
        <td id="L652" class="blob-num js-line-number" data-line-number="652"></td>
        <td id="LC652" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		if (second == 0)</span></td>
      </tr>
      <tr>
        <td id="L653" class="blob-num js-line-number" data-line-number="653"></td>
        <td id="LC653" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		{</span></td>
      </tr>
      <tr>
        <td id="L654" class="blob-num js-line-number" data-line-number="654"></td>
        <td id="LC654" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			position = 0.0f;</span></td>
      </tr>
      <tr>
        <td id="L655" class="blob-num js-line-number" data-line-number="655"></td>
        <td id="LC655" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		}</span></td>
      </tr>
      <tr>
        <td id="L656" class="blob-num js-line-number" data-line-number="656"></td>
        <td id="LC656" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		else</span></td>
      </tr>
      <tr>
        <td id="L657" class="blob-num js-line-number" data-line-number="657"></td>
        <td id="LC657" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		{</span></td>
      </tr>
      <tr>
        <td id="L658" class="blob-num js-line-number" data-line-number="658"></td>
        <td id="LC658" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			position += speed * 5280.0f / 3600.0f * 0.3048f;</span></td>
      </tr>
      <tr>
        <td id="L659" class="blob-num js-line-number" data-line-number="659"></td>
        <td id="LC659" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		}</span></td>
      </tr>
      <tr>
        <td id="L660" class="blob-num js-line-number" data-line-number="660"></td>
        <td id="LC660" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		data.position = position;</span></td>
      </tr>
      <tr>
        <td id="L661" class="blob-num js-line-number" data-line-number="661"></td>
        <td id="LC661" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		pDrivingCycleSample-&gt;dataVector.push_back(data);</span></td>
      </tr>
      <tr>
        <td id="L662" class="blob-num js-line-number" data-line-number="662"></td>
        <td id="LC662" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	}</span></td>
      </tr>
      <tr>
        <td id="L663" class="blob-num js-line-number" data-line-number="663"></td>
        <td id="LC663" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	fclose(pFile);</span></td>
      </tr>
      <tr>
        <td id="L664" class="blob-num js-line-number" data-line-number="664"></td>
        <td id="LC664" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L665" class="blob-num js-line-number" data-line-number="665"></td>
        <td id="LC665" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	if (vehicle_type &lt; MAX_VEHICLE_TYPE_SIZE)</span></td>
      </tr>
      <tr>
        <td id="L666" class="blob-num js-line-number" data-line-number="666"></td>
        <td id="LC666" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	{</span></td>
      </tr>
      <tr>
        <td id="L667" class="blob-num js-line-number" data-line-number="667"></td>
        <td id="LC667" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		for (int i=0;i&lt;drivingCycleSamples.size();i++)</span></td>
      </tr>
      <tr>
        <td id="L668" class="blob-num js-line-number" data-line-number="668"></td>
        <td id="LC668" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		{</span></td>
      </tr>
      <tr>
        <td id="L669" class="blob-num js-line-number" data-line-number="669"></td>
        <td id="LC669" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			int segment_id = 0;</span></td>
      </tr>
      <tr>
        <td id="L670" class="blob-num js-line-number" data-line-number="670"></td>
        <td id="LC670" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			for (int j=0;j&lt;drivingCycleSamples[i]-&gt;dataVector.size();j=j+drivingCycleSamples[i]-&gt;defaultTimeSpan)</span></td>
      </tr>
      <tr>
        <td id="L671" class="blob-num js-line-number" data-line-number="671"></td>
        <td id="LC671" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			{</span></td>
      </tr>
      <tr>
        <td id="L672" class="blob-num js-line-number" data-line-number="672"></td>
        <td id="LC672" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//				float initial_speed = drivingCycleSamples[i]-&gt;dataVector[i].speed;</span></td>
      </tr>
      <tr>
        <td id="L673" class="blob-num js-line-number" data-line-number="673"></td>
        <td id="LC673" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//				int speed_bin = GetDrivingCycleVehicleSpeedBin(initial_speed);</span></td>
      </tr>
      <tr>
        <td id="L674" class="blob-num js-line-number" data-line-number="674"></td>
        <td id="LC674" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//				trajectoryMatrixBin[vehicle_type][speed_bin].AddToMatrix(*drivingCycleSamples[i],segment_id);</span></td>
      </tr>
      <tr>
        <td id="L675" class="blob-num js-line-number" data-line-number="675"></td>
        <td id="LC675" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//				segment_id++;</span></td>
      </tr>
      <tr>
        <td id="L676" class="blob-num js-line-number" data-line-number="676"></td>
        <td id="LC676" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			}</span></td>
      </tr>
      <tr>
        <td id="L677" class="blob-num js-line-number" data-line-number="677"></td>
        <td id="LC677" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		}</span></td>
      </tr>
      <tr>
        <td id="L678" class="blob-num js-line-number" data-line-number="678"></td>
        <td id="LC678" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	}</span></td>
      </tr>
      <tr>
        <td id="L679" class="blob-num js-line-number" data-line-number="679"></td>
        <td id="LC679" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//}</span></td>
      </tr>
      <tr>
        <td id="L680" class="blob-num js-line-number" data-line-number="680"></td>
        <td id="LC680" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L681" class="blob-num js-line-number" data-line-number="681"></td>
        <td id="LC681" class="blob-code blob-code-inner js-file-line"><span class="pl-k">int</span> <span class="pl-en">SmoothVehicleAcceleration</span>(<span class="pl-k">float</span> link_ff_speed, <span class="pl-k">float</span> link_length, <span class="pl-k">float</span> initial_speed, <span class="pl-k">int</span> start_time, <span class="pl-k">int</span> end_time, <span class="pl-k">float</span>* position, <span class="pl-k">int</span> size)</td>
      </tr>
      <tr>
        <td id="L682" class="blob-num js-line-number" data-line-number="682"></td>
        <td id="LC682" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L683" class="blob-num js-line-number" data-line-number="683"></td>
        <td id="LC683" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">const</span> <span class="pl-k">float</span> Acceleration_Rate = <span class="pl-c1">7</span>.<span class="pl-c1">5f</span>; <span class="pl-c">//8mph/s</span></td>
      </tr>
      <tr>
        <td id="L684" class="blob-num js-line-number" data-line-number="684"></td>
        <td id="LC684" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">const</span> <span class="pl-k">float</span> Deceleration_Rate = <span class="pl-c1">5</span>.<span class="pl-c1">0f</span>; <span class="pl-c">//3.3mph/s</span></td>
      </tr>
      <tr>
        <td id="L685" class="blob-num js-line-number" data-line-number="685"></td>
        <td id="LC685" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> speed_above_ff_speed = <span class="pl-c1">10.0</span>; <span class="pl-c">//mph</span></td>
      </tr>
      <tr>
        <td id="L686" class="blob-num js-line-number" data-line-number="686"></td>
        <td id="LC686" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L687" class="blob-num js-line-number" data-line-number="687"></td>
        <td id="LC687" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//t1: acceleration time: from initial_speed to link_ff_speed + speed_above_ff_speed</span></td>
      </tr>
      <tr>
        <td id="L688" class="blob-num js-line-number" data-line-number="688"></td>
        <td id="LC688" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//t2: constant cruising time </span></td>
      </tr>
      <tr>
        <td id="L689" class="blob-num js-line-number" data-line-number="689"></td>
        <td id="LC689" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//t3: deceleration time: from link_ff_speed + speed_above_ff_speed to link_ff_speed</span></td>
      </tr>
      <tr>
        <td id="L690" class="blob-num js-line-number" data-line-number="690"></td>
        <td id="LC690" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L691" class="blob-num js-line-number" data-line-number="691"></td>
        <td id="LC691" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> t1 = (link_ff_speed + speed_above_ff_speed - initial_speed) / Acceleration_Rate; <span class="pl-c">// in second</span></td>
      </tr>
      <tr>
        <td id="L692" class="blob-num js-line-number" data-line-number="692"></td>
        <td id="LC692" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> t3 = speed_above_ff_speed / Deceleration_Rate; <span class="pl-c">// in second</span></td>
      </tr>
      <tr>
        <td id="L693" class="blob-num js-line-number" data-line-number="693"></td>
        <td id="LC693" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> t2 = (link_ff_speed * <span class="pl-c1">5280</span>.<span class="pl-c1">0f</span> / <span class="pl-c1">3600</span>.<span class="pl-c1">0f</span> * (t1 + t3) - (initial_speed * <span class="pl-c1">5280</span>.<span class="pl-c1">0f</span> / <span class="pl-c1">3600</span>.<span class="pl-c1">0f</span> * t1 + <span class="pl-c1">0</span>.<span class="pl-c1">5f</span> * Acceleration_Rate * <span class="pl-c1">5280</span>.<span class="pl-c1">0f</span> / <span class="pl-c1">3600</span>.<span class="pl-c1">0f</span> * t1 * t1 + (link_ff_speed + speed_above_ff_speed) * (<span class="pl-c1">5280</span>.<span class="pl-c1">0f</span> / <span class="pl-c1">3600</span>.<span class="pl-c1">0f</span>) * t3 - <span class="pl-c1">0</span>.<span class="pl-c1">5f</span> * Deceleration_Rate * (<span class="pl-c1">5280</span>.<span class="pl-c1">0f</span> / <span class="pl-c1">3600</span>.<span class="pl-c1">0f</span>) * t3 * t3)) / (speed_above_ff_speed * <span class="pl-c1">5280</span>.<span class="pl-c1">0f</span> / <span class="pl-c1">3600</span>.<span class="pl-c1">0f</span>);</td>
      </tr>
      <tr>
        <td id="L694" class="blob-num js-line-number" data-line-number="694"></td>
        <td id="LC694" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L695" class="blob-num js-line-number" data-line-number="695"></td>
        <td id="LC695" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> ((<span class="pl-k">int</span>)((t1 + t2 + t3) * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND) &lt;= (end_time - start_time))</td>
      </tr>
      <tr>
        <td id="L696" class="blob-num js-line-number" data-line-number="696"></td>
        <td id="LC696" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L697" class="blob-num js-line-number" data-line-number="697"></td>
        <td id="LC697" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">int</span> end_of_duration_1 = start_time + (<span class="pl-k">int</span>)(t1 * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND);</td>
      </tr>
      <tr>
        <td id="L698" class="blob-num js-line-number" data-line-number="698"></td>
        <td id="LC698" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">int</span> end_of_duration_2 = end_of_duration_1 + (<span class="pl-k">int</span>)(t2 * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND);</td>
      </tr>
      <tr>
        <td id="L699" class="blob-num js-line-number" data-line-number="699"></td>
        <td id="LC699" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">int</span> end_of_duration_3 = end_of_duration_2 + (<span class="pl-k">int</span>)(t3 * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND);</td>
      </tr>
      <tr>
        <td id="L700" class="blob-num js-line-number" data-line-number="700"></td>
        <td id="LC700" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">for</span> (<span class="pl-k">int</span> t = start_time; t &lt; end_of_duration_1; t++)</td>
      </tr>
      <tr>
        <td id="L701" class="blob-num js-line-number" data-line-number="701"></td>
        <td id="LC701" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L702" class="blob-num js-line-number" data-line-number="702"></td>
        <td id="LC702" class="blob-code blob-code-inner js-file-line">			position[t % size] = initial_speed * <span class="pl-c1">5280</span>.<span class="pl-c1">0f</span> / <span class="pl-c1">3600</span>.<span class="pl-c1">0f</span> + <span class="pl-c1">0</span>.<span class="pl-c1">5f</span> * Acceleration_Rate * <span class="pl-c1">5280</span>.<span class="pl-c1">0f</span> / <span class="pl-c1">3600</span>.<span class="pl-c1">0f</span> * (t - start_time + <span class="pl-c1">1</span>) * (t - start_time) / (NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND);</td>
      </tr>
      <tr>
        <td id="L703" class="blob-num js-line-number" data-line-number="703"></td>
        <td id="LC703" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L704" class="blob-num js-line-number" data-line-number="704"></td>
        <td id="LC704" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L705" class="blob-num js-line-number" data-line-number="705"></td>
        <td id="LC705" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">for</span> (<span class="pl-k">int</span> t = end_of_duration_1; t &lt; end_of_duration_2; t++)</td>
      </tr>
      <tr>
        <td id="L706" class="blob-num js-line-number" data-line-number="706"></td>
        <td id="LC706" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L707" class="blob-num js-line-number" data-line-number="707"></td>
        <td id="LC707" class="blob-code blob-code-inner js-file-line">			position[t % size] = position[end_of_duration_1 -<span class="pl-c1">1</span>] + (link_ff_speed + speed_above_ff_speed) * <span class="pl-c1">5280</span>.<span class="pl-c1">0f</span> / <span class="pl-c1">3600</span>.<span class="pl-c1">0f</span> * (t - end_of_duration_1 + <span class="pl-c1">1</span>) / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;</td>
      </tr>
      <tr>
        <td id="L708" class="blob-num js-line-number" data-line-number="708"></td>
        <td id="LC708" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L709" class="blob-num js-line-number" data-line-number="709"></td>
        <td id="LC709" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L710" class="blob-num js-line-number" data-line-number="710"></td>
        <td id="LC710" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">for</span> (<span class="pl-k">int</span> t = end_of_duration_2; t &lt; end_of_duration_3; t++)</td>
      </tr>
      <tr>
        <td id="L711" class="blob-num js-line-number" data-line-number="711"></td>
        <td id="LC711" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L712" class="blob-num js-line-number" data-line-number="712"></td>
        <td id="LC712" class="blob-code blob-code-inner js-file-line">			position[t % size] = position[end_of_duration_2 - <span class="pl-c1">1</span>] + (link_ff_speed + speed_above_ff_speed) * <span class="pl-c1">5280</span>.<span class="pl-c1">0f</span> / <span class="pl-c1">3600</span>.<span class="pl-c1">0f</span> * (t - end_of_duration_2 + <span class="pl-c1">1</span>) / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND - <span class="pl-c1">0</span>.<span class="pl-c1">5f</span> * Deceleration_Rate * <span class="pl-c1">5280</span>.<span class="pl-c1">0f</span> / <span class="pl-c1">3600</span>.<span class="pl-c1">0f</span> * (t - end_of_duration_2) * (t - end_of_duration_2) / (NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND);</td>
      </tr>
      <tr>
        <td id="L713" class="blob-num js-line-number" data-line-number="713"></td>
        <td id="LC713" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L714" class="blob-num js-line-number" data-line-number="714"></td>
        <td id="LC714" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L715" class="blob-num js-line-number" data-line-number="715"></td>
        <td id="LC715" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">return</span> (t1+t2+t3);</td>
      </tr>
      <tr>
        <td id="L716" class="blob-num js-line-number" data-line-number="716"></td>
        <td id="LC716" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L717" class="blob-num js-line-number" data-line-number="717"></td>
        <td id="LC717" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L718" class="blob-num js-line-number" data-line-number="718"></td>
        <td id="LC718" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L719" class="blob-num js-line-number" data-line-number="719"></td>
        <td id="LC719" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">return</span> <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L720" class="blob-num js-line-number" data-line-number="720"></td>
        <td id="LC720" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L721" class="blob-num js-line-number" data-line-number="721"></td>
        <td id="LC721" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L722" class="blob-num js-line-number" data-line-number="722"></td>
        <td id="LC722" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L723" class="blob-num js-line-number" data-line-number="723"></td>
        <td id="LC723" class="blob-code blob-code-inner js-file-line"><span class="pl-k">void</span> <span class="pl-en">SetupOperatingModeVector</span>()</td>
      </tr>
      <tr>
        <td id="L724" class="blob-num js-line-number" data-line-number="724"></td>
        <td id="LC724" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L725" class="blob-num js-line-number" data-line-number="725"></td>
        <td id="LC725" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_0_25mph][VSP_LT0] = <span class="pl-c1">11</span>;</td>
      </tr>
      <tr>
        <td id="L726" class="blob-num js-line-number" data-line-number="726"></td>
        <td id="LC726" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_0_25mph][VSP_0_3] = <span class="pl-c1">12</span>;</td>
      </tr>
      <tr>
        <td id="L727" class="blob-num js-line-number" data-line-number="727"></td>
        <td id="LC727" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_0_25mph][VSP_3_6] = <span class="pl-c1">13</span>;</td>
      </tr>
      <tr>
        <td id="L728" class="blob-num js-line-number" data-line-number="728"></td>
        <td id="LC728" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_0_25mph][VSP_6_9] = <span class="pl-c1">14</span>;</td>
      </tr>
      <tr>
        <td id="L729" class="blob-num js-line-number" data-line-number="729"></td>
        <td id="LC729" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_0_25mph][VSP_9_12] = <span class="pl-c1">15</span>;</td>
      </tr>
      <tr>
        <td id="L730" class="blob-num js-line-number" data-line-number="730"></td>
        <td id="LC730" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_0_25mph][VSP_12_18] = <span class="pl-c1">16</span>;</td>
      </tr>
      <tr>
        <td id="L731" class="blob-num js-line-number" data-line-number="731"></td>
        <td id="LC731" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_0_25mph][VSP_18_24] = <span class="pl-c1">16</span>;</td>
      </tr>
      <tr>
        <td id="L732" class="blob-num js-line-number" data-line-number="732"></td>
        <td id="LC732" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_0_25mph][VSP_24_30] = <span class="pl-c1">16</span>;</td>
      </tr>
      <tr>
        <td id="L733" class="blob-num js-line-number" data-line-number="733"></td>
        <td id="LC733" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_0_25mph][VSP_GT30] = <span class="pl-c1">16</span>;</td>
      </tr>
      <tr>
        <td id="L734" class="blob-num js-line-number" data-line-number="734"></td>
        <td id="LC734" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L735" class="blob-num js-line-number" data-line-number="735"></td>
        <td id="LC735" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_25_50mph][VSP_LT0] = <span class="pl-c1">21</span>;</td>
      </tr>
      <tr>
        <td id="L736" class="blob-num js-line-number" data-line-number="736"></td>
        <td id="LC736" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_25_50mph][VSP_0_3] = <span class="pl-c1">22</span>;</td>
      </tr>
      <tr>
        <td id="L737" class="blob-num js-line-number" data-line-number="737"></td>
        <td id="LC737" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_25_50mph][VSP_3_6] = <span class="pl-c1">23</span>;</td>
      </tr>
      <tr>
        <td id="L738" class="blob-num js-line-number" data-line-number="738"></td>
        <td id="LC738" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_25_50mph][VSP_6_9] = <span class="pl-c1">24</span>;</td>
      </tr>
      <tr>
        <td id="L739" class="blob-num js-line-number" data-line-number="739"></td>
        <td id="LC739" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_25_50mph][VSP_9_12] = <span class="pl-c1">25</span>;</td>
      </tr>
      <tr>
        <td id="L740" class="blob-num js-line-number" data-line-number="740"></td>
        <td id="LC740" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_25_50mph][VSP_12_18] = <span class="pl-c1">27</span>;</td>
      </tr>
      <tr>
        <td id="L741" class="blob-num js-line-number" data-line-number="741"></td>
        <td id="LC741" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_25_50mph][VSP_18_24] = <span class="pl-c1">28</span>;</td>
      </tr>
      <tr>
        <td id="L742" class="blob-num js-line-number" data-line-number="742"></td>
        <td id="LC742" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_25_50mph][VSP_24_30] = <span class="pl-c1">29</span>;</td>
      </tr>
      <tr>
        <td id="L743" class="blob-num js-line-number" data-line-number="743"></td>
        <td id="LC743" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_25_50mph][VSP_GT30] = <span class="pl-c1">30</span>;</td>
      </tr>
      <tr>
        <td id="L744" class="blob-num js-line-number" data-line-number="744"></td>
        <td id="LC744" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L745" class="blob-num js-line-number" data-line-number="745"></td>
        <td id="LC745" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_GT50mph][VSP_LT0] = <span class="pl-c1">33</span>;</td>
      </tr>
      <tr>
        <td id="L746" class="blob-num js-line-number" data-line-number="746"></td>
        <td id="LC746" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_GT50mph][VSP_0_3] = <span class="pl-c1">33</span>;</td>
      </tr>
      <tr>
        <td id="L747" class="blob-num js-line-number" data-line-number="747"></td>
        <td id="LC747" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_GT50mph][VSP_3_6] = <span class="pl-c1">33</span>;</td>
      </tr>
      <tr>
        <td id="L748" class="blob-num js-line-number" data-line-number="748"></td>
        <td id="LC748" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_GT50mph][VSP_6_9] = <span class="pl-c1">35</span>;</td>
      </tr>
      <tr>
        <td id="L749" class="blob-num js-line-number" data-line-number="749"></td>
        <td id="LC749" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_GT50mph][VSP_9_12] = <span class="pl-c1">35</span>;</td>
      </tr>
      <tr>
        <td id="L750" class="blob-num js-line-number" data-line-number="750"></td>
        <td id="LC750" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_GT50mph][VSP_12_18] = <span class="pl-c1">37</span>;</td>
      </tr>
      <tr>
        <td id="L751" class="blob-num js-line-number" data-line-number="751"></td>
        <td id="LC751" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_GT50mph][VSP_18_24] = <span class="pl-c1">38</span>;</td>
      </tr>
      <tr>
        <td id="L752" class="blob-num js-line-number" data-line-number="752"></td>
        <td id="LC752" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_GT50mph][VSP_24_30] = <span class="pl-c1">39</span>;</td>
      </tr>
      <tr>
        <td id="L753" class="blob-num js-line-number" data-line-number="753"></td>
        <td id="LC753" class="blob-code blob-code-inner js-file-line">	OperatingModeMap[VSP_GT50mph][VSP_GT30] = <span class="pl-c1">40</span>;</td>
      </tr>
      <tr>
        <td id="L754" class="blob-num js-line-number" data-line-number="754"></td>
        <td id="LC754" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L755" class="blob-num js-line-number" data-line-number="755"></td>
        <td id="LC755" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L756" class="blob-num js-line-number" data-line-number="756"></td>
        <td id="LC756" class="blob-code blob-code-inner js-file-line"><span class="pl-k">int</span> <span class="pl-en">GetOperatingMode</span>(<span class="pl-k">float</span> vsp, <span class="pl-k">float</span> s_mph)</td>
      </tr>
      <tr>
        <td id="L757" class="blob-num js-line-number" data-line-number="757"></td>
        <td id="LC757" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L758" class="blob-num js-line-number" data-line-number="758"></td>
        <td id="LC758" class="blob-code blob-code-inner js-file-line">	SPEED_BIN SpeedBinNo = <span class="pl-c1">GetSpeedBinNo</span>(s_mph);</td>
      </tr>
      <tr>
        <td id="L759" class="blob-num js-line-number" data-line-number="759"></td>
        <td id="LC759" class="blob-code blob-code-inner js-file-line">	VSP_BIN VSPBinNo =  <span class="pl-c1">GetVSPBinNo</span>(vsp, s_mph);</td>
      </tr>
      <tr>
        <td id="L760" class="blob-num js-line-number" data-line-number="760"></td>
        <td id="LC760" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L761" class="blob-num js-line-number" data-line-number="761"></td>
        <td id="LC761" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> OPBin =  OperatingModeMap[SpeedBinNo][VSPBinNo];</td>
      </tr>
      <tr>
        <td id="L762" class="blob-num js-line-number" data-line-number="762"></td>
        <td id="LC762" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">return</span> OPBin;</td>
      </tr>
      <tr>
        <td id="L763" class="blob-num js-line-number" data-line-number="763"></td>
        <td id="LC763" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L764" class="blob-num js-line-number" data-line-number="764"></td>
        <td id="LC764" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L765" class="blob-num js-line-number" data-line-number="765"></td>
        <td id="LC765" class="blob-code blob-code-inner js-file-line"><span class="pl-k">int</span> <span class="pl-en">ComputeOperatingModeFromSpeed</span>(<span class="pl-k">float</span> &amp;vsp, <span class="pl-k">float</span> v <span class="pl-c">/*meter per second*/</span>,<span class="pl-k">float</span> a, <span class="pl-k">float</span> grade = <span class="pl-c1">0</span>, <span class="pl-k">int</span> vehicle_type =<span class="pl-c1">1</span>)</td>
      </tr>
      <tr>
        <td id="L766" class="blob-num js-line-number" data-line-number="766"></td>
        <td id="LC766" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L767" class="blob-num js-line-number" data-line-number="767"></td>
        <td id="LC767" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> vehicle_type_no = vehicle_type -<span class="pl-c1">1</span>; <span class="pl-c">// start from 0</span></td>
      </tr>
      <tr>
        <td id="L768" class="blob-num js-line-number" data-line-number="768"></td>
        <td id="LC768" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">double</span> TermA = g_VehicleTypeVector[vehicle_type_no].<span class="pl-smi">rollingTermA</span> ;</td>
      </tr>
      <tr>
        <td id="L769" class="blob-num js-line-number" data-line-number="769"></td>
        <td id="LC769" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">double</span> TermB = g_VehicleTypeVector[vehicle_type_no].<span class="pl-smi">rotatingTermB</span> ;</td>
      </tr>
      <tr>
        <td id="L770" class="blob-num js-line-number" data-line-number="770"></td>
        <td id="LC770" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">double</span> TermC = g_VehicleTypeVector[vehicle_type_no].<span class="pl-smi">dragTermC</span>  ;</td>
      </tr>
      <tr>
        <td id="L771" class="blob-num js-line-number" data-line-number="771"></td>
        <td id="LC771" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">double</span> Mass =  g_VehicleTypeVector[vehicle_type_no].<span class="pl-smi">sourceMass</span>;</td>
      </tr>
      <tr>
        <td id="L772" class="blob-num js-line-number" data-line-number="772"></td>
        <td id="LC772" class="blob-code blob-code-inner js-file-line">	vsp = (TermA*v + TermB*v*v +  TermC*v*v*v + v*a*Mass)/Mass;</td>
      </tr>
      <tr>
        <td id="L773" class="blob-num js-line-number" data-line-number="773"></td>
        <td id="LC773" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//vsp = TermA/Mass*v + TermB/Mass*v*v+  TermC/Mass*v*v*v;</span></td>
      </tr>
      <tr>
        <td id="L774" class="blob-num js-line-number" data-line-number="774"></td>
        <td id="LC774" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> speed_mph = v * <span class="pl-c1">2</span>.<span class="pl-c1">23693629f</span>;  <span class="pl-c">//3600 seconds / 1606 meters per hour</span></td>
      </tr>
      <tr>
        <td id="L775" class="blob-num js-line-number" data-line-number="775"></td>
        <td id="LC775" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L776" class="blob-num js-line-number" data-line-number="776"></td>
        <td id="LC776" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> OpMode = <span class="pl-c1">GetOperatingMode</span>(vsp,speed_mph);</td>
      </tr>
      <tr>
        <td id="L777" class="blob-num js-line-number" data-line-number="777"></td>
        <td id="LC777" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L778" class="blob-num js-line-number" data-line-number="778"></td>
        <td id="LC778" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">return</span> OpMode; </td>
      </tr>
      <tr>
        <td id="L779" class="blob-num js-line-number" data-line-number="779"></td>
        <td id="LC779" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L780" class="blob-num js-line-number" data-line-number="780"></td>
        <td id="LC780" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L781" class="blob-num js-line-number" data-line-number="781"></td>
        <td id="LC781" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L782" class="blob-num js-line-number" data-line-number="782"></td>
        <td id="LC782" class="blob-code blob-code-inner js-file-line"><span class="pl-k">bool</span> <span class="pl-en">vehicleCF_sort_function</span> (VehicleCFData i,VehicleCFData j) </td>
      </tr>
      <tr>
        <td id="L783" class="blob-num js-line-number" data-line-number="783"></td>
        <td id="LC783" class="blob-code blob-code-inner js-file-line">{ </td>
      </tr>
      <tr>
        <td id="L784" class="blob-num js-line-number" data-line-number="784"></td>
        <td id="LC784" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">return</span> (i.<span class="pl-smi">StartTime_in_SimulationInterval</span> &lt; j.<span class="pl-smi">StartTime_in_SimulationInterval</span> );</td>
      </tr>
      <tr>
        <td id="L785" class="blob-num js-line-number" data-line-number="785"></td>
        <td id="LC785" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L786" class="blob-num js-line-number" data-line-number="786"></td>
        <td id="LC786" class="blob-code blob-code-inner js-file-line"><span class="pl-k">bool</span> <span class="pl-en">matchMicroTrip</span>(<span class="pl-k">int</span> vehicle_type, <span class="pl-k">int</span> start_time, <span class="pl-k">float</span> start_speed, <span class="pl-k">float</span> start_position, vector&lt;<span class="pl-k">float</span>&gt;&amp; speedArray, vector&lt;<span class="pl-k">float</span>&gt;&amp; position, </td>
      </tr>
      <tr>
        <td id="L787" class="blob-num js-line-number" data-line-number="787"></td>
        <td id="LC787" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">int</span> arraySize, DrivingCycleIdentifier&amp; matchedDrivingCycleIdentifier, <span class="pl-k">int</span>&amp; exactSecond)</td>
      </tr>
      <tr>
        <td id="L788" class="blob-num js-line-number" data-line-number="788"></td>
        <td id="LC788" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L789" class="blob-num js-line-number" data-line-number="789"></td>
        <td id="LC789" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//int speed_bin = GetDrivingCycleVehicleSpeedBin(start_speed);</span></td>
      </tr>
      <tr>
        <td id="L790" class="blob-num js-line-number" data-line-number="790"></td>
        <td id="LC790" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//TrajectoryMatrix&amp; trajMatrix = trajectoryMatrixBin[speed_bin][vehicle_type];</span></td>
      </tr>
      <tr>
        <td id="L791" class="blob-num js-line-number" data-line-number="791"></td>
        <td id="LC791" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//bool isMatch = false;</span></td>
      </tr>
      <tr>
        <td id="L792" class="blob-num js-line-number" data-line-number="792"></td>
        <td id="LC792" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L793" class="blob-num js-line-number" data-line-number="793"></td>
        <td id="LC793" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//int t = 0;</span></td>
      </tr>
      <tr>
        <td id="L794" class="blob-num js-line-number" data-line-number="794"></td>
        <td id="LC794" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//for (int n = start_time; n &lt; arraySize &amp;&amp; t &lt; trajMatrix.timeWindowSize; n++,t++)</span></td>
      </tr>
      <tr>
        <td id="L795" class="blob-num js-line-number" data-line-number="795"></td>
        <td id="LC795" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//{</span></td>
      </tr>
      <tr>
        <td id="L796" class="blob-num js-line-number" data-line-number="796"></td>
        <td id="LC796" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//	int upperBound = min((int)(position[n] - start_position) + 10, trajMatrix.spaceWindowSize);</span></td>
      </tr>
      <tr>
        <td id="L797" class="blob-num js-line-number" data-line-number="797"></td>
        <td id="LC797" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//	int lowerBound = max(0, upperBound - 20);</span></td>
      </tr>
      <tr>
        <td id="L798" class="blob-num js-line-number" data-line-number="798"></td>
        <td id="LC798" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L799" class="blob-num js-line-number" data-line-number="799"></td>
        <td id="LC799" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//	for (int m = lowerBound; m &lt; upperBound; m++)</span></td>
      </tr>
      <tr>
        <td id="L800" class="blob-num js-line-number" data-line-number="800"></td>
        <td id="LC800" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//	{</span></td>
      </tr>
      <tr>
        <td id="L801" class="blob-num js-line-number" data-line-number="801"></td>
        <td id="LC801" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//		vector&lt;DrivingCycleIdentifier&gt;* pDrivingCycleIdentifierVector = trajMatrix.GetIdentifier(t, m);</span></td>
      </tr>
      <tr>
        <td id="L802" class="blob-num js-line-number" data-line-number="802"></td>
        <td id="LC802" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L803" class="blob-num js-line-number" data-line-number="803"></td>
        <td id="LC803" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//		if (pDrivingCycleIdentifierVector &amp;&amp; pDrivingCycleIdentifierVector-&gt;size() &gt; 0)</span></td>
      </tr>
      <tr>
        <td id="L804" class="blob-num js-line-number" data-line-number="804"></td>
        <td id="LC804" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//		{</span></td>
      </tr>
      <tr>
        <td id="L805" class="blob-num js-line-number" data-line-number="805"></td>
        <td id="LC805" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//			float targetSpeed = speedArray[n+1];</span></td>
      </tr>
      <tr>
        <td id="L806" class="blob-num js-line-number" data-line-number="806"></td>
        <td id="LC806" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//			for (int k = 0; k &lt; pDrivingCycleIdentifierVector-&gt;size();k++)</span></td>
      </tr>
      <tr>
        <td id="L807" class="blob-num js-line-number" data-line-number="807"></td>
        <td id="LC807" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//			{</span></td>
      </tr>
      <tr>
        <td id="L808" class="blob-num js-line-number" data-line-number="808"></td>
        <td id="LC808" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//				float speed = drivingCycleMap[vehicle_type][(*pDrivingCycleIdentifierVector)[k].cycle_id -1]-&gt;GetSpeedByTime((*pDrivingCycleIdentifierVector)[k].segment_id,n - start_time);</span></td>
      </tr>
      <tr>
        <td id="L809" class="blob-num js-line-number" data-line-number="809"></td>
        <td id="LC809" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//				if (abs(speed - targetSpeed) &lt; 5)</span></td>
      </tr>
      <tr>
        <td id="L810" class="blob-num js-line-number" data-line-number="810"></td>
        <td id="LC810" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//				{</span></td>
      </tr>
      <tr>
        <td id="L811" class="blob-num js-line-number" data-line-number="811"></td>
        <td id="LC811" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//					matchedDrivingCycleIdentifier = (*pDrivingCycleIdentifierVector)[0];</span></td>
      </tr>
      <tr>
        <td id="L812" class="blob-num js-line-number" data-line-number="812"></td>
        <td id="LC812" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//					exactSecond = n - start_time;</span></td>
      </tr>
      <tr>
        <td id="L813" class="blob-num js-line-number" data-line-number="813"></td>
        <td id="LC813" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//					return true;</span></td>
      </tr>
      <tr>
        <td id="L814" class="blob-num js-line-number" data-line-number="814"></td>
        <td id="LC814" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//				}</span></td>
      </tr>
      <tr>
        <td id="L815" class="blob-num js-line-number" data-line-number="815"></td>
        <td id="LC815" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//			}</span></td>
      </tr>
      <tr>
        <td id="L816" class="blob-num js-line-number" data-line-number="816"></td>
        <td id="LC816" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//		}</span></td>
      </tr>
      <tr>
        <td id="L817" class="blob-num js-line-number" data-line-number="817"></td>
        <td id="LC817" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//	}</span></td>
      </tr>
      <tr>
        <td id="L818" class="blob-num js-line-number" data-line-number="818"></td>
        <td id="LC818" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L819" class="blob-num js-line-number" data-line-number="819"></td>
        <td id="LC819" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//}</span></td>
      </tr>
      <tr>
        <td id="L820" class="blob-num js-line-number" data-line-number="820"></td>
        <td id="LC820" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L821" class="blob-num js-line-number" data-line-number="821"></td>
        <td id="LC821" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//return false;</span></td>
      </tr>
      <tr>
        <td id="L822" class="blob-num js-line-number" data-line-number="822"></td>
        <td id="LC822" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">///*for (int i=0;i&lt;trajMatrix.timeWindowSize;i++)</span></td>
      </tr>
      <tr>
        <td id="L823" class="blob-num js-line-number" data-line-number="823"></td>
        <td id="LC823" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//{</span></td>
      </tr>
      <tr>
        <td id="L824" class="blob-num js-line-number" data-line-number="824"></td>
        <td id="LC824" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//	for (int j=0;j&lt;trajMatrix.spaceWindowSize;j++)</span></td>
      </tr>
      <tr>
        <td id="L825" class="blob-num js-line-number" data-line-number="825"></td>
        <td id="LC825" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//	{</span></td>
      </tr>
      <tr>
        <td id="L826" class="blob-num js-line-number" data-line-number="826"></td>
        <td id="LC826" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L827" class="blob-num js-line-number" data-line-number="827"></td>
        <td id="LC827" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//	}</span></td>
      </tr>
      <tr>
        <td id="L828" class="blob-num js-line-number" data-line-number="828"></td>
        <td id="LC828" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//}*/</span></td>
      </tr>
      <tr>
        <td id="L829" class="blob-num js-line-number" data-line-number="829"></td>
        <td id="LC829" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">return</span> <span class="pl-c1">true</span>;</td>
      </tr>
      <tr>
        <td id="L830" class="blob-num js-line-number" data-line-number="830"></td>
        <td id="LC830" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L831" class="blob-num js-line-number" data-line-number="831"></td>
        <td id="LC831" class="blob-code blob-code-inner js-file-line"><span class="pl-k">void</span> <span class="pl-en">ReconstructTrajectory</span>(<span class="pl-k">int</span> vehicle_type, std::map&lt;<span class="pl-k">int</span>, VehicleSpeedProfileData&gt;&amp; speedProfile)</td>
      </tr>
      <tr>
        <td id="L832" class="blob-num js-line-number" data-line-number="832"></td>
        <td id="LC832" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L833" class="blob-num js-line-number" data-line-number="833"></td>
        <td id="LC833" class="blob-code blob-code-inner js-file-line">	std::map&lt;<span class="pl-k">int</span>, VehicleSpeedProfileData&gt;::iterator iter = speedProfile.<span class="pl-c1">begin</span>();</td>
      </tr>
      <tr>
        <td id="L834" class="blob-num js-line-number" data-line-number="834"></td>
        <td id="LC834" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> size = speedProfile.<span class="pl-c1">size</span>();</td>
      </tr>
      <tr>
        <td id="L835" class="blob-num js-line-number" data-line-number="835"></td>
        <td id="LC835" class="blob-code blob-code-inner js-file-line">	vector&lt;<span class="pl-k">float</span>&gt; speedArray;</td>
      </tr>
      <tr>
        <td id="L836" class="blob-num js-line-number" data-line-number="836"></td>
        <td id="LC836" class="blob-code blob-code-inner js-file-line">	vector&lt;<span class="pl-k">float</span>&gt; position;</td>
      </tr>
      <tr>
        <td id="L837" class="blob-num js-line-number" data-line-number="837"></td>
        <td id="LC837" class="blob-code blob-code-inner js-file-line">	position.<span class="pl-c1">push_back</span>(<span class="pl-c1">0</span>.<span class="pl-c1">0f</span>);</td>
      </tr>
      <tr>
        <td id="L838" class="blob-num js-line-number" data-line-number="838"></td>
        <td id="LC838" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">bool</span> isUpdated = <span class="pl-c1">false</span>;</td>
      </tr>
      <tr>
        <td id="L839" class="blob-num js-line-number" data-line-number="839"></td>
        <td id="LC839" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L840" class="blob-num js-line-number" data-line-number="840"></td>
        <td id="LC840" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">for</span> (<span class="pl-k">int</span> i=<span class="pl-c1">0</span>;iter != speedProfile.<span class="pl-c1">end</span>();iter++,i++)</td>
      </tr>
      <tr>
        <td id="L841" class="blob-num js-line-number" data-line-number="841"></td>
        <td id="LC841" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L842" class="blob-num js-line-number" data-line-number="842"></td>
        <td id="LC842" class="blob-code blob-code-inner js-file-line">		speedArray.<span class="pl-c1">push_back</span>(iter-&gt;second.<span class="pl-smi">Speed</span>);</td>
      </tr>
      <tr>
        <td id="L843" class="blob-num js-line-number" data-line-number="843"></td>
        <td id="LC843" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">if</span> (i == <span class="pl-c1">0</span>)</td>
      </tr>
      <tr>
        <td id="L844" class="blob-num js-line-number" data-line-number="844"></td>
        <td id="LC844" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L845" class="blob-num js-line-number" data-line-number="845"></td>
        <td id="LC845" class="blob-code blob-code-inner js-file-line">			position[<span class="pl-c1">0</span>] = <span class="pl-c1">0</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L846" class="blob-num js-line-number" data-line-number="846"></td>
        <td id="LC846" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L847" class="blob-num js-line-number" data-line-number="847"></td>
        <td id="LC847" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L848" class="blob-num js-line-number" data-line-number="848"></td>
        <td id="LC848" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L849" class="blob-num js-line-number" data-line-number="849"></td>
        <td id="LC849" class="blob-code blob-code-inner js-file-line">			position.<span class="pl-c1">push_back</span>(position[i-<span class="pl-c1">1</span>] + iter-&gt;second.<span class="pl-smi">Speed</span> * <span class="pl-c1">0</span>.<span class="pl-c1">44704f</span>);</td>
      </tr>
      <tr>
        <td id="L850" class="blob-num js-line-number" data-line-number="850"></td>
        <td id="LC850" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L851" class="blob-num js-line-number" data-line-number="851"></td>
        <td id="LC851" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L852" class="blob-num js-line-number" data-line-number="852"></td>
        <td id="LC852" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L853" class="blob-num js-line-number" data-line-number="853"></td>
        <td id="LC853" class="blob-code blob-code-inner js-file-line">	vector&lt;<span class="pl-k">int</span>&gt; transitionPoints;</td>
      </tr>
      <tr>
        <td id="L854" class="blob-num js-line-number" data-line-number="854"></td>
        <td id="LC854" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">for</span> (<span class="pl-k">int</span> i=<span class="pl-c1">1</span>;i&lt;size;i++)</td>
      </tr>
      <tr>
        <td id="L855" class="blob-num js-line-number" data-line-number="855"></td>
        <td id="LC855" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L856" class="blob-num js-line-number" data-line-number="856"></td>
        <td id="LC856" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">if</span> (<span class="pl-c1">abs</span>(speedArray[i] - speedArray[i-<span class="pl-c1">1</span>]) &gt; <span class="pl-c1">10</span>)</td>
      </tr>
      <tr>
        <td id="L857" class="blob-num js-line-number" data-line-number="857"></td>
        <td id="LC857" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L858" class="blob-num js-line-number" data-line-number="858"></td>
        <td id="LC858" class="blob-code blob-code-inner js-file-line">			transitionPoints.<span class="pl-c1">push_back</span>(i);</td>
      </tr>
      <tr>
        <td id="L859" class="blob-num js-line-number" data-line-number="859"></td>
        <td id="LC859" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L860" class="blob-num js-line-number" data-line-number="860"></td>
        <td id="LC860" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L861" class="blob-num js-line-number" data-line-number="861"></td>
        <td id="LC861" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L862" class="blob-num js-line-number" data-line-number="862"></td>
        <td id="LC862" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (transitionPoints.<span class="pl-c1">size</span>() &gt; <span class="pl-c1">0</span>)</td>
      </tr>
      <tr>
        <td id="L863" class="blob-num js-line-number" data-line-number="863"></td>
        <td id="LC863" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L864" class="blob-num js-line-number" data-line-number="864"></td>
        <td id="LC864" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">for</span> (<span class="pl-k">int</span> i=<span class="pl-c1">0</span>;i&lt;transitionPoints.<span class="pl-c1">size</span>();i++)</td>
      </tr>
      <tr>
        <td id="L865" class="blob-num js-line-number" data-line-number="865"></td>
        <td id="LC865" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L866" class="blob-num js-line-number" data-line-number="866"></td>
        <td id="LC866" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">int</span> t = transitionPoints[i];</td>
      </tr>
      <tr>
        <td id="L867" class="blob-num js-line-number" data-line-number="867"></td>
        <td id="LC867" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">int</span> exactSecond;</td>
      </tr>
      <tr>
        <td id="L868" class="blob-num js-line-number" data-line-number="868"></td>
        <td id="LC868" class="blob-code blob-code-inner js-file-line">			DrivingCycleIdentifier drivingCycleIdentifier;</td>
      </tr>
      <tr>
        <td id="L869" class="blob-num js-line-number" data-line-number="869"></td>
        <td id="LC869" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (<span class="pl-c1">matchMicroTrip</span>(vehicle_type,t,speedArray[t-<span class="pl-c1">1</span>], position[t-<span class="pl-c1">1</span>],speedArray,position, size, drivingCycleIdentifier, exactSecond))</td>
      </tr>
      <tr>
        <td id="L870" class="blob-num js-line-number" data-line-number="870"></td>
        <td id="LC870" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L871" class="blob-num js-line-number" data-line-number="871"></td>
        <td id="LC871" class="blob-code blob-code-inner js-file-line">				std::cout &lt;&lt; <span class="pl-s"><span class="pl-pds">&quot;</span>==================Microtrip matched!====================<span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>;</td>
      </tr>
      <tr>
        <td id="L872" class="blob-num js-line-number" data-line-number="872"></td>
        <td id="LC872" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (i != transitionPoints.<span class="pl-c1">size</span>() - <span class="pl-c1">1</span>)</td>
      </tr>
      <tr>
        <td id="L873" class="blob-num js-line-number" data-line-number="873"></td>
        <td id="LC873" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L874" class="blob-num js-line-number" data-line-number="874"></td>
        <td id="LC874" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">if</span> (t + exactSecond &lt; transitionPoints[i+<span class="pl-c1">1</span>])</td>
      </tr>
      <tr>
        <td id="L875" class="blob-num js-line-number" data-line-number="875"></td>
        <td id="LC875" class="blob-code blob-code-inner js-file-line">					{						</td>
      </tr>
      <tr>
        <td id="L876" class="blob-num js-line-number" data-line-number="876"></td>
        <td id="LC876" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">for</span> (<span class="pl-k">int</span> n=<span class="pl-c1">0</span>;n&lt;exactSecond;n++)</td>
      </tr>
      <tr>
        <td id="L877" class="blob-num js-line-number" data-line-number="877"></td>
        <td id="LC877" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L878" class="blob-num js-line-number" data-line-number="878"></td>
        <td id="LC878" class="blob-code blob-code-inner js-file-line">							speedArray[i-<span class="pl-c1">1</span>] = drivingCycleMap[vehicle_type][drivingCycleIdentifier.<span class="pl-smi">cycle_id</span>]-&gt;<span class="pl-c1">GetSpeedByTime</span>(drivingCycleIdentifier.<span class="pl-smi">segment_id</span>,n);</td>
      </tr>
      <tr>
        <td id="L879" class="blob-num js-line-number" data-line-number="879"></td>
        <td id="LC879" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L880" class="blob-num js-line-number" data-line-number="880"></td>
        <td id="LC880" class="blob-code blob-code-inner js-file-line">						isUpdated = <span class="pl-c1">true</span>;</td>
      </tr>
      <tr>
        <td id="L881" class="blob-num js-line-number" data-line-number="881"></td>
        <td id="LC881" class="blob-code blob-code-inner js-file-line">					}</td>
      </tr>
      <tr>
        <td id="L882" class="blob-num js-line-number" data-line-number="882"></td>
        <td id="LC882" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L883" class="blob-num js-line-number" data-line-number="883"></td>
        <td id="LC883" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L884" class="blob-num js-line-number" data-line-number="884"></td>
        <td id="LC884" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L885" class="blob-num js-line-number" data-line-number="885"></td>
        <td id="LC885" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">if</span> (t + exactSecond &lt; size)</td>
      </tr>
      <tr>
        <td id="L886" class="blob-num js-line-number" data-line-number="886"></td>
        <td id="LC886" class="blob-code blob-code-inner js-file-line">					{</td>
      </tr>
      <tr>
        <td id="L887" class="blob-num js-line-number" data-line-number="887"></td>
        <td id="LC887" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">for</span> (<span class="pl-k">int</span> n=<span class="pl-c1">0</span>;n&lt;exactSecond;n++)</td>
      </tr>
      <tr>
        <td id="L888" class="blob-num js-line-number" data-line-number="888"></td>
        <td id="LC888" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L889" class="blob-num js-line-number" data-line-number="889"></td>
        <td id="LC889" class="blob-code blob-code-inner js-file-line">							speedArray[i-<span class="pl-c1">1</span>] = drivingCycleMap[vehicle_type][drivingCycleIdentifier.<span class="pl-smi">cycle_id</span>]-&gt;<span class="pl-c1">GetSpeedByTime</span>(drivingCycleIdentifier.<span class="pl-smi">segment_id</span>,n);</td>
      </tr>
      <tr>
        <td id="L890" class="blob-num js-line-number" data-line-number="890"></td>
        <td id="LC890" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L891" class="blob-num js-line-number" data-line-number="891"></td>
        <td id="LC891" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L892" class="blob-num js-line-number" data-line-number="892"></td>
        <td id="LC892" class="blob-code blob-code-inner js-file-line">						isUpdated = <span class="pl-c1">true</span>;</td>
      </tr>
      <tr>
        <td id="L893" class="blob-num js-line-number" data-line-number="893"></td>
        <td id="LC893" class="blob-code blob-code-inner js-file-line">					}</td>
      </tr>
      <tr>
        <td id="L894" class="blob-num js-line-number" data-line-number="894"></td>
        <td id="LC894" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L895" class="blob-num js-line-number" data-line-number="895"></td>
        <td id="LC895" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L896" class="blob-num js-line-number" data-line-number="896"></td>
        <td id="LC896" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L897" class="blob-num js-line-number" data-line-number="897"></td>
        <td id="LC897" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L898" class="blob-num js-line-number" data-line-number="898"></td>
        <td id="LC898" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L899" class="blob-num js-line-number" data-line-number="899"></td>
        <td id="LC899" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L900" class="blob-num js-line-number" data-line-number="900"></td>
        <td id="LC900" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L901" class="blob-num js-line-number" data-line-number="901"></td>
        <td id="LC901" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L902" class="blob-num js-line-number" data-line-number="902"></td>
        <td id="LC902" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L903" class="blob-num js-line-number" data-line-number="903"></td>
        <td id="LC903" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (isUpdated)</td>
      </tr>
      <tr>
        <td id="L904" class="blob-num js-line-number" data-line-number="904"></td>
        <td id="LC904" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L905" class="blob-num js-line-number" data-line-number="905"></td>
        <td id="LC905" class="blob-code blob-code-inner js-file-line">		iter = speedProfile.<span class="pl-c1">begin</span>();</td>
      </tr>
      <tr>
        <td id="L906" class="blob-num js-line-number" data-line-number="906"></td>
        <td id="LC906" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">for</span> (<span class="pl-k">int</span> i=<span class="pl-c1">0</span>;iter != speedProfile.<span class="pl-c1">end</span>();iter++,i++)</td>
      </tr>
      <tr>
        <td id="L907" class="blob-num js-line-number" data-line-number="907"></td>
        <td id="LC907" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L908" class="blob-num js-line-number" data-line-number="908"></td>
        <td id="LC908" class="blob-code blob-code-inner js-file-line">			iter-&gt;second.<span class="pl-smi">Speed</span> = speedArray[i];</td>
      </tr>
      <tr>
        <td id="L909" class="blob-num js-line-number" data-line-number="909"></td>
        <td id="LC909" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L910" class="blob-num js-line-number" data-line-number="910"></td>
        <td id="LC910" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L911" class="blob-num js-line-number" data-line-number="911"></td>
        <td id="LC911" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L912" class="blob-num js-line-number" data-line-number="912"></td>
        <td id="LC912" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L913" class="blob-num js-line-number" data-line-number="913"></td>
        <td id="LC913" class="blob-code blob-code-inner js-file-line"><span class="pl-k">enum</span> TransitionType {ACC, DEC};</td>
      </tr>
      <tr>
        <td id="L914" class="blob-num js-line-number" data-line-number="914"></td>
        <td id="LC914" class="blob-code blob-code-inner js-file-line"><span class="pl-k">typedef</span> <span class="pl-k">struct</span></td>
      </tr>
      <tr>
        <td id="L915" class="blob-num js-line-number" data-line-number="915"></td>
        <td id="LC915" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L916" class="blob-num js-line-number" data-line-number="916"></td>
        <td id="LC916" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> t;</td>
      </tr>
      <tr>
        <td id="L917" class="blob-num js-line-number" data-line-number="917"></td>
        <td id="LC917" class="blob-code blob-code-inner js-file-line">	TransitionType type;</td>
      </tr>
      <tr>
        <td id="L918" class="blob-num js-line-number" data-line-number="918"></td>
        <td id="LC918" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L919" class="blob-num js-line-number" data-line-number="919"></td>
        <td id="LC919" class="blob-code blob-code-inner js-file-line">} TransitionPoint;</td>
      </tr>
      <tr>
        <td id="L920" class="blob-num js-line-number" data-line-number="920"></td>
        <td id="LC920" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L921" class="blob-num js-line-number" data-line-number="921"></td>
        <td id="LC921" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L922" class="blob-num js-line-number" data-line-number="922"></td>
        <td id="LC922" class="blob-code blob-code-inner js-file-line"><span class="pl-c">/* Acceleration and Deceleration Range</span></td>
      </tr>
      <tr>
        <td id="L923" class="blob-num js-line-number" data-line-number="923"></td>
        <td id="LC923" class="blob-code blob-code-inner js-file-line"><span class="pl-c">0-1			-2		0.6</span></td>
      </tr>
      <tr>
        <td id="L924" class="blob-num js-line-number" data-line-number="924"></td>
        <td id="LC924" class="blob-code blob-code-inner js-file-line"><span class="pl-c">1-5			-3.7	2.4</span></td>
      </tr>
      <tr>
        <td id="L925" class="blob-num js-line-number" data-line-number="925"></td>
        <td id="LC925" class="blob-code blob-code-inner js-file-line"><span class="pl-c">5-10		-5.1	5.6</span></td>
      </tr>
      <tr>
        <td id="L926" class="blob-num js-line-number" data-line-number="926"></td>
        <td id="LC926" class="blob-code blob-code-inner js-file-line"><span class="pl-c">10-20		-4.5	5</span></td>
      </tr>
      <tr>
        <td id="L927" class="blob-num js-line-number" data-line-number="927"></td>
        <td id="LC927" class="blob-code blob-code-inner js-file-line"><span class="pl-c">20-30		-3.7	3.6</span></td>
      </tr>
      <tr>
        <td id="L928" class="blob-num js-line-number" data-line-number="928"></td>
        <td id="LC928" class="blob-code blob-code-inner js-file-line"><span class="pl-c">30-40		-2.9	2.7</span></td>
      </tr>
      <tr>
        <td id="L929" class="blob-num js-line-number" data-line-number="929"></td>
        <td id="LC929" class="blob-code blob-code-inner js-file-line"><span class="pl-c">40-50		-1.6	1.7</span></td>
      </tr>
      <tr>
        <td id="L930" class="blob-num js-line-number" data-line-number="930"></td>
        <td id="LC930" class="blob-code blob-code-inner js-file-line"><span class="pl-c">50-60		-1		1</span></td>
      </tr>
      <tr>
        <td id="L931" class="blob-num js-line-number" data-line-number="931"></td>
        <td id="LC931" class="blob-code blob-code-inner js-file-line"><span class="pl-c">60-70		-1		1</span></td>
      </tr>
      <tr>
        <td id="L932" class="blob-num js-line-number" data-line-number="932"></td>
        <td id="LC932" class="blob-code blob-code-inner js-file-line"><span class="pl-c">70 and up	-1		1</span></td>
      </tr>
      <tr>
        <td id="L933" class="blob-num js-line-number" data-line-number="933"></td>
        <td id="LC933" class="blob-code blob-code-inner js-file-line"><span class="pl-c"></span></td>
      </tr>
      <tr>
        <td id="L934" class="blob-num js-line-number" data-line-number="934"></td>
        <td id="LC934" class="blob-code blob-code-inner js-file-line"><span class="pl-c">*/</span></td>
      </tr>
      <tr>
        <td id="L935" class="blob-num js-line-number" data-line-number="935"></td>
        <td id="LC935" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L936" class="blob-num js-line-number" data-line-number="936"></td>
        <td id="LC936" class="blob-code blob-code-inner js-file-line"><span class="pl-k">float</span> maxAcceleration = (<span class="pl-c1">0</span>.<span class="pl-c1">6f</span>, <span class="pl-c1">2</span>.<span class="pl-c1">4f</span>, <span class="pl-c1">5</span>.<span class="pl-c1">6f</span>, <span class="pl-c1">5</span>.<span class="pl-c1">0f</span>, <span class="pl-c1">3</span>.<span class="pl-c1">6f</span>, <span class="pl-c1">2</span>.<span class="pl-c1">7f</span>, <span class="pl-c1">1</span>.<span class="pl-c1">7f</span>, <span class="pl-c1">1</span>.<span class="pl-c1">0f</span>, <span class="pl-c1">1</span>.<span class="pl-c1">0f</span>, <span class="pl-c1">1</span>.<span class="pl-c1">0f</span>);</td>
      </tr>
      <tr>
        <td id="L937" class="blob-num js-line-number" data-line-number="937"></td>
        <td id="LC937" class="blob-code blob-code-inner js-file-line"><span class="pl-k">float</span> maxDeceleration = (-<span class="pl-c1">2</span>.<span class="pl-c1">0f</span>, -<span class="pl-c1">3</span>.<span class="pl-c1">7f</span>, -<span class="pl-c1">5</span>.<span class="pl-c1">1f</span>, -<span class="pl-c1">4</span>.<span class="pl-c1">5f</span>, -<span class="pl-c1">3</span>.<span class="pl-c1">7f</span>, -<span class="pl-c1">2</span>.<span class="pl-c1">9f</span>, -<span class="pl-c1">1</span>.<span class="pl-c1">6f</span>, -<span class="pl-c1">1</span>.<span class="pl-c1">0f</span>, -<span class="pl-c1">1</span>.<span class="pl-c1">0f</span>, -<span class="pl-c1">1</span>.<span class="pl-c1">0f</span>);</td>
      </tr>
      <tr>
        <td id="L938" class="blob-num js-line-number" data-line-number="938"></td>
        <td id="LC938" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L939" class="blob-num js-line-number" data-line-number="939"></td>
        <td id="LC939" class="blob-code blob-code-inner js-file-line"><span class="pl-k">float</span> <span class="pl-en">maxAccelerationBySpeed</span>(<span class="pl-k">float</span> speedInMiles)</td>
      </tr>
      <tr>
        <td id="L940" class="blob-num js-line-number" data-line-number="940"></td>
        <td id="LC940" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L941" class="blob-num js-line-number" data-line-number="941"></td>
        <td id="LC941" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (speedInMiles &lt; <span class="pl-c1">1</span>) <span class="pl-k">return</span> <span class="pl-c1">0</span>.<span class="pl-c1">6f</span>;</td>
      </tr>
      <tr>
        <td id="L942" class="blob-num js-line-number" data-line-number="942"></td>
        <td id="LC942" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (speedInMiles &lt; <span class="pl-c1">5</span>) <span class="pl-k">return</span> <span class="pl-c1">2</span>.<span class="pl-c1">4f</span>;</td>
      </tr>
      <tr>
        <td id="L943" class="blob-num js-line-number" data-line-number="943"></td>
        <td id="LC943" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (speedInMiles &lt; <span class="pl-c1">10</span>) <span class="pl-k">return</span> <span class="pl-c1">5</span>.<span class="pl-c1">6f</span>;</td>
      </tr>
      <tr>
        <td id="L944" class="blob-num js-line-number" data-line-number="944"></td>
        <td id="LC944" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (speedInMiles &lt; <span class="pl-c1">20</span>) <span class="pl-k">return</span> <span class="pl-c1">5</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L945" class="blob-num js-line-number" data-line-number="945"></td>
        <td id="LC945" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (speedInMiles &lt; <span class="pl-c1">30</span>) <span class="pl-k">return</span> <span class="pl-c1">3</span>.<span class="pl-c1">6f</span>;</td>
      </tr>
      <tr>
        <td id="L946" class="blob-num js-line-number" data-line-number="946"></td>
        <td id="LC946" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (speedInMiles &lt; <span class="pl-c1">40</span>) <span class="pl-k">return</span> <span class="pl-c1">2</span>.<span class="pl-c1">7f</span>;</td>
      </tr>
      <tr>
        <td id="L947" class="blob-num js-line-number" data-line-number="947"></td>
        <td id="LC947" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (speedInMiles &lt; <span class="pl-c1">50</span>) <span class="pl-k">return</span> <span class="pl-c1">1</span>.<span class="pl-c1">7f</span>;</td>
      </tr>
      <tr>
        <td id="L948" class="blob-num js-line-number" data-line-number="948"></td>
        <td id="LC948" class="blob-code blob-code-inner js-file-line">	</td>
      </tr>
      <tr>
        <td id="L949" class="blob-num js-line-number" data-line-number="949"></td>
        <td id="LC949" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">return</span> <span class="pl-c1">1</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L950" class="blob-num js-line-number" data-line-number="950"></td>
        <td id="LC950" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L951" class="blob-num js-line-number" data-line-number="951"></td>
        <td id="LC951" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L952" class="blob-num js-line-number" data-line-number="952"></td>
        <td id="LC952" class="blob-code blob-code-inner js-file-line"><span class="pl-k">float</span> <span class="pl-en">maxDecelerationBySpeed</span>(<span class="pl-k">float</span> speedInMiles)</td>
      </tr>
      <tr>
        <td id="L953" class="blob-num js-line-number" data-line-number="953"></td>
        <td id="LC953" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L954" class="blob-num js-line-number" data-line-number="954"></td>
        <td id="LC954" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (speedInMiles &lt; <span class="pl-c1">1</span>) <span class="pl-k">return</span> -<span class="pl-c1">2</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L955" class="blob-num js-line-number" data-line-number="955"></td>
        <td id="LC955" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (speedInMiles &lt; <span class="pl-c1">5</span>) <span class="pl-k">return</span> -<span class="pl-c1">3</span>.<span class="pl-c1">7f</span>;</td>
      </tr>
      <tr>
        <td id="L956" class="blob-num js-line-number" data-line-number="956"></td>
        <td id="LC956" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (speedInMiles &lt; <span class="pl-c1">10</span>) <span class="pl-k">return</span> -<span class="pl-c1">5</span>.<span class="pl-c1">1f</span>;</td>
      </tr>
      <tr>
        <td id="L957" class="blob-num js-line-number" data-line-number="957"></td>
        <td id="LC957" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (speedInMiles &lt; <span class="pl-c1">20</span>) <span class="pl-k">return</span> -<span class="pl-c1">4</span>.<span class="pl-c1">5f</span>;</td>
      </tr>
      <tr>
        <td id="L958" class="blob-num js-line-number" data-line-number="958"></td>
        <td id="LC958" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (speedInMiles &lt; <span class="pl-c1">30</span>) <span class="pl-k">return</span> -<span class="pl-c1">3</span>.<span class="pl-c1">7f</span>;</td>
      </tr>
      <tr>
        <td id="L959" class="blob-num js-line-number" data-line-number="959"></td>
        <td id="LC959" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (speedInMiles &lt; <span class="pl-c1">40</span>) <span class="pl-k">return</span> -<span class="pl-c1">2</span>.<span class="pl-c1">9f</span>;</td>
      </tr>
      <tr>
        <td id="L960" class="blob-num js-line-number" data-line-number="960"></td>
        <td id="LC960" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (speedInMiles &lt; <span class="pl-c1">50</span>) <span class="pl-k">return</span> -<span class="pl-c1">1</span>.<span class="pl-c1">6f</span>;</td>
      </tr>
      <tr>
        <td id="L961" class="blob-num js-line-number" data-line-number="961"></td>
        <td id="LC961" class="blob-code blob-code-inner js-file-line">	</td>
      </tr>
      <tr>
        <td id="L962" class="blob-num js-line-number" data-line-number="962"></td>
        <td id="LC962" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">return</span> -<span class="pl-c1">1</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L963" class="blob-num js-line-number" data-line-number="963"></td>
        <td id="LC963" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L964" class="blob-num js-line-number" data-line-number="964"></td>
        <td id="LC964" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L965" class="blob-num js-line-number" data-line-number="965"></td>
        <td id="LC965" class="blob-code blob-code-inner js-file-line"><span class="pl-k">void</span> <span class="pl-en">MovingAverage</span>(std::vector&lt;<span class="pl-k">float</span>&gt;&amp; p)</td>
      </tr>
      <tr>
        <td id="L966" class="blob-num js-line-number" data-line-number="966"></td>
        <td id="LC966" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L967" class="blob-num js-line-number" data-line-number="967"></td>
        <td id="LC967" class="blob-code blob-code-inner js-file-line">	<span class="pl-c1">size_t</span> sz = p.<span class="pl-c1">size</span>();</td>
      </tr>
      <tr>
        <td id="L968" class="blob-num js-line-number" data-line-number="968"></td>
        <td id="LC968" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L969" class="blob-num js-line-number" data-line-number="969"></td>
        <td id="LC969" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (sz &gt;= <span class="pl-c1">7</span>)</td>
      </tr>
      <tr>
        <td id="L970" class="blob-num js-line-number" data-line-number="970"></td>
        <td id="LC970" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L971" class="blob-num js-line-number" data-line-number="971"></td>
        <td id="LC971" class="blob-code blob-code-inner js-file-line">		p[<span class="pl-c1">0</span>] = p[<span class="pl-c1">0</span>];</td>
      </tr>
      <tr>
        <td id="L972" class="blob-num js-line-number" data-line-number="972"></td>
        <td id="LC972" class="blob-code blob-code-inner js-file-line">		p[<span class="pl-c1">1</span>] = (p[<span class="pl-c1">0</span>] + p[<span class="pl-c1">1</span>] + p[<span class="pl-c1">2</span>]) / <span class="pl-c1">3</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L973" class="blob-num js-line-number" data-line-number="973"></td>
        <td id="LC973" class="blob-code blob-code-inner js-file-line">		p[<span class="pl-c1">2</span>] = (p[<span class="pl-c1">0</span>] + p[<span class="pl-c1">1</span>] + p[<span class="pl-c1">2</span>] + p[<span class="pl-c1">3</span>] + p[<span class="pl-c1">4</span>]) / <span class="pl-c1">5</span>.<span class="pl-c1">0f</span>;		</td>
      </tr>
      <tr>
        <td id="L974" class="blob-num js-line-number" data-line-number="974"></td>
        <td id="LC974" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L975" class="blob-num js-line-number" data-line-number="975"></td>
        <td id="LC975" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">for</span> (<span class="pl-k">int</span> i = <span class="pl-c1">3</span>; i &lt;= sz - <span class="pl-c1">4</span>; i++)</td>
      </tr>
      <tr>
        <td id="L976" class="blob-num js-line-number" data-line-number="976"></td>
        <td id="LC976" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L977" class="blob-num js-line-number" data-line-number="977"></td>
        <td id="LC977" class="blob-code blob-code-inner js-file-line">			p[i] = (p[i - <span class="pl-c1">3</span>] + p[i - <span class="pl-c1">2</span>] + p[i - <span class="pl-c1">1</span>] + p[i] + p[i + <span class="pl-c1">1</span>] + p[i + <span class="pl-c1">2</span>] + p[i + <span class="pl-c1">3</span>]) / <span class="pl-c1">7</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L978" class="blob-num js-line-number" data-line-number="978"></td>
        <td id="LC978" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L979" class="blob-num js-line-number" data-line-number="979"></td>
        <td id="LC979" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L980" class="blob-num js-line-number" data-line-number="980"></td>
        <td id="LC980" class="blob-code blob-code-inner js-file-line">		p[sz - <span class="pl-c1">3</span>] = (p[sz - <span class="pl-c1">5</span>] + p[sz - <span class="pl-c1">4</span>] + p[sz - <span class="pl-c1">3</span>] + p[sz - <span class="pl-c1">2</span>] + p[sz - <span class="pl-c1">1</span>] ) / <span class="pl-c1">5</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L981" class="blob-num js-line-number" data-line-number="981"></td>
        <td id="LC981" class="blob-code blob-code-inner js-file-line">		p[sz - <span class="pl-c1">2</span>] = (p[sz - <span class="pl-c1">3</span>] + p[sz - <span class="pl-c1">2</span>] + p[sz - <span class="pl-c1">1</span>]) / <span class="pl-c1">3</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L982" class="blob-num js-line-number" data-line-number="982"></td>
        <td id="LC982" class="blob-code blob-code-inner js-file-line">		p[sz - <span class="pl-c1">1</span>] = p[sz - <span class="pl-c1">1</span>];</td>
      </tr>
      <tr>
        <td id="L983" class="blob-num js-line-number" data-line-number="983"></td>
        <td id="LC983" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L984" class="blob-num js-line-number" data-line-number="984"></td>
        <td id="LC984" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L985" class="blob-num js-line-number" data-line-number="985"></td>
        <td id="LC985" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">return</span>;</td>
      </tr>
      <tr>
        <td id="L986" class="blob-num js-line-number" data-line-number="986"></td>
        <td id="LC986" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L987" class="blob-num js-line-number" data-line-number="987"></td>
        <td id="LC987" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L988" class="blob-num js-line-number" data-line-number="988"></td>
        <td id="LC988" class="blob-code blob-code-inner js-file-line"><span class="pl-k">void</span> <span class="pl-en">MovingAverage</span>(<span class="pl-k">float</span>* p, <span class="pl-k">int</span> sz)</td>
      </tr>
      <tr>
        <td id="L989" class="blob-num js-line-number" data-line-number="989"></td>
        <td id="LC989" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L990" class="blob-num js-line-number" data-line-number="990"></td>
        <td id="LC990" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (p == <span class="pl-c1">NULL</span> || sz == <span class="pl-c1">0</span>)</td>
      </tr>
      <tr>
        <td id="L991" class="blob-num js-line-number" data-line-number="991"></td>
        <td id="LC991" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L992" class="blob-num js-line-number" data-line-number="992"></td>
        <td id="LC992" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">return</span>;</td>
      </tr>
      <tr>
        <td id="L993" class="blob-num js-line-number" data-line-number="993"></td>
        <td id="LC993" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L994" class="blob-num js-line-number" data-line-number="994"></td>
        <td id="LC994" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L995" class="blob-num js-line-number" data-line-number="995"></td>
        <td id="LC995" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (sz &gt;= <span class="pl-c1">7</span>)</td>
      </tr>
      <tr>
        <td id="L996" class="blob-num js-line-number" data-line-number="996"></td>
        <td id="LC996" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L997" class="blob-num js-line-number" data-line-number="997"></td>
        <td id="LC997" class="blob-code blob-code-inner js-file-line">		p[<span class="pl-c1">0</span>] = p[<span class="pl-c1">0</span>];</td>
      </tr>
      <tr>
        <td id="L998" class="blob-num js-line-number" data-line-number="998"></td>
        <td id="LC998" class="blob-code blob-code-inner js-file-line">		p[<span class="pl-c1">1</span>] = (p[<span class="pl-c1">0</span>] + p[<span class="pl-c1">1</span>] + p[<span class="pl-c1">2</span>]) / <span class="pl-c1">3</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L999" class="blob-num js-line-number" data-line-number="999"></td>
        <td id="LC999" class="blob-code blob-code-inner js-file-line">		p[<span class="pl-c1">2</span>] = (p[<span class="pl-c1">0</span>] + p[<span class="pl-c1">1</span>] + p[<span class="pl-c1">2</span>] + p[<span class="pl-c1">3</span>] + p[<span class="pl-c1">4</span>]) / <span class="pl-c1">5</span>.<span class="pl-c1">0f</span>;		</td>
      </tr>
      <tr>
        <td id="L1000" class="blob-num js-line-number" data-line-number="1000"></td>
        <td id="LC1000" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1001" class="blob-num js-line-number" data-line-number="1001"></td>
        <td id="LC1001" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">for</span> (<span class="pl-k">int</span> i = <span class="pl-c1">3</span>; i &lt;= sz - <span class="pl-c1">4</span>; i++)</td>
      </tr>
      <tr>
        <td id="L1002" class="blob-num js-line-number" data-line-number="1002"></td>
        <td id="LC1002" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L1003" class="blob-num js-line-number" data-line-number="1003"></td>
        <td id="LC1003" class="blob-code blob-code-inner js-file-line">			p[i] = (p[i - <span class="pl-c1">3</span>] + p[i - <span class="pl-c1">2</span>] + p[i - <span class="pl-c1">1</span>] + p[i] + p[i + <span class="pl-c1">1</span>] + p[i + <span class="pl-c1">2</span>] + p[i + <span class="pl-c1">3</span>]) / <span class="pl-c1">7</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L1004" class="blob-num js-line-number" data-line-number="1004"></td>
        <td id="LC1004" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L1005" class="blob-num js-line-number" data-line-number="1005"></td>
        <td id="LC1005" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1006" class="blob-num js-line-number" data-line-number="1006"></td>
        <td id="LC1006" class="blob-code blob-code-inner js-file-line">		p[sz - <span class="pl-c1">3</span>] = (p[sz - <span class="pl-c1">5</span>] + p[sz - <span class="pl-c1">4</span>] + p[sz - <span class="pl-c1">3</span>] + p[sz - <span class="pl-c1">2</span>] + p[sz - <span class="pl-c1">1</span>] ) / <span class="pl-c1">5</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L1007" class="blob-num js-line-number" data-line-number="1007"></td>
        <td id="LC1007" class="blob-code blob-code-inner js-file-line">		p[sz - <span class="pl-c1">2</span>] = (p[sz - <span class="pl-c1">3</span>] + p[sz - <span class="pl-c1">2</span>] + p[sz - <span class="pl-c1">1</span>]) / <span class="pl-c1">3</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L1008" class="blob-num js-line-number" data-line-number="1008"></td>
        <td id="LC1008" class="blob-code blob-code-inner js-file-line">		p[sz - <span class="pl-c1">1</span>] = p[sz - <span class="pl-c1">1</span>];</td>
      </tr>
      <tr>
        <td id="L1009" class="blob-num js-line-number" data-line-number="1009"></td>
        <td id="LC1009" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L1010" class="blob-num js-line-number" data-line-number="1010"></td>
        <td id="LC1010" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1011" class="blob-num js-line-number" data-line-number="1011"></td>
        <td id="LC1011" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">return</span>;</td>
      </tr>
      <tr>
        <td id="L1012" class="blob-num js-line-number" data-line-number="1012"></td>
        <td id="LC1012" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L1013" class="blob-num js-line-number" data-line-number="1013"></td>
        <td id="LC1013" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1014" class="blob-num js-line-number" data-line-number="1014"></td>
        <td id="LC1014" class="blob-code blob-code-inner js-file-line">vector&lt;<span class="pl-k">float</span>&gt; <span class="pl-en">SpeedUpTo</span>(<span class="pl-k">int</span> vehicle_id, <span class="pl-k">float</span> initial_speed, <span class="pl-k">float</span> targetSpeed)</td>
      </tr>
      <tr>
        <td id="L1015" class="blob-num js-line-number" data-line-number="1015"></td>
        <td id="LC1015" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L1016" class="blob-num js-line-number" data-line-number="1016"></td>
        <td id="LC1016" class="blob-code blob-code-inner js-file-line">	</td>
      </tr>
      <tr>
        <td id="L1017" class="blob-num js-line-number" data-line-number="1017"></td>
        <td id="LC1017" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> speed = initial_speed;</td>
      </tr>
      <tr>
        <td id="L1018" class="blob-num js-line-number" data-line-number="1018"></td>
        <td id="LC1018" class="blob-code blob-code-inner js-file-line">	vector&lt;<span class="pl-k">float</span>&gt; speedVector;</td>
      </tr>
      <tr>
        <td id="L1019" class="blob-num js-line-number" data-line-number="1019"></td>
        <td id="LC1019" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//if (initial_speed &gt; targetSpeed) return speedVector;</span></td>
      </tr>
      <tr>
        <td id="L1020" class="blob-num js-line-number" data-line-number="1020"></td>
        <td id="LC1020" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1021" class="blob-num js-line-number" data-line-number="1021"></td>
        <td id="LC1021" class="blob-code blob-code-inner js-file-line">	<span class="pl-c1">srand</span>(vehicle_id);</td>
      </tr>
      <tr>
        <td id="L1022" class="blob-num js-line-number" data-line-number="1022"></td>
        <td id="LC1022" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1023" class="blob-num js-line-number" data-line-number="1023"></td>
        <td id="LC1023" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> currAccRate = <span class="pl-c1">maxAccelerationBySpeed</span>(initial_speed) * (<span class="pl-k">float</span>)(<span class="pl-c1">rand</span>() / <span class="pl-c1">double</span>(RAND_MAX));</td>
      </tr>
      <tr>
        <td id="L1024" class="blob-num js-line-number" data-line-number="1024"></td>
        <td id="LC1024" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1025" class="blob-num js-line-number" data-line-number="1025"></td>
        <td id="LC1025" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">while</span> (speed + currAccRate &lt; targetSpeed)</td>
      </tr>
      <tr>
        <td id="L1026" class="blob-num js-line-number" data-line-number="1026"></td>
        <td id="LC1026" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L1027" class="blob-num js-line-number" data-line-number="1027"></td>
        <td id="LC1027" class="blob-code blob-code-inner js-file-line">		speed += currAccRate;</td>
      </tr>
      <tr>
        <td id="L1028" class="blob-num js-line-number" data-line-number="1028"></td>
        <td id="LC1028" class="blob-code blob-code-inner js-file-line">		speedVector.<span class="pl-c1">push_back</span>(speed);</td>
      </tr>
      <tr>
        <td id="L1029" class="blob-num js-line-number" data-line-number="1029"></td>
        <td id="LC1029" class="blob-code blob-code-inner js-file-line">		currAccRate = <span class="pl-c1">maxAccelerationBySpeed</span>(speed) * (<span class="pl-k">float</span>)(<span class="pl-c1">rand</span>() / <span class="pl-c1">double</span>(RAND_MAX));</td>
      </tr>
      <tr>
        <td id="L1030" class="blob-num js-line-number" data-line-number="1030"></td>
        <td id="LC1030" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L1031" class="blob-num js-line-number" data-line-number="1031"></td>
        <td id="LC1031" class="blob-code blob-code-inner js-file-line">	speedVector.<span class="pl-c1">push_back</span>(targetSpeed);</td>
      </tr>
      <tr>
        <td id="L1032" class="blob-num js-line-number" data-line-number="1032"></td>
        <td id="LC1032" class="blob-code blob-code-inner js-file-line">	<span class="pl-c1">MovingAverage</span>(speedVector);</td>
      </tr>
      <tr>
        <td id="L1033" class="blob-num js-line-number" data-line-number="1033"></td>
        <td id="LC1033" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">return</span> speedVector;</td>
      </tr>
      <tr>
        <td id="L1034" class="blob-num js-line-number" data-line-number="1034"></td>
        <td id="LC1034" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L1035" class="blob-num js-line-number" data-line-number="1035"></td>
        <td id="LC1035" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1036" class="blob-num js-line-number" data-line-number="1036"></td>
        <td id="LC1036" class="blob-code blob-code-inner js-file-line">vector&lt;<span class="pl-k">float</span>&gt; <span class="pl-en">SlowDownTo</span>(<span class="pl-k">int</span> vehicle_id, <span class="pl-k">float</span> initial_speed, <span class="pl-k">float</span> targetSpeed)</td>
      </tr>
      <tr>
        <td id="L1037" class="blob-num js-line-number" data-line-number="1037"></td>
        <td id="LC1037" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L1038" class="blob-num js-line-number" data-line-number="1038"></td>
        <td id="LC1038" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> speed = initial_speed;</td>
      </tr>
      <tr>
        <td id="L1039" class="blob-num js-line-number" data-line-number="1039"></td>
        <td id="LC1039" class="blob-code blob-code-inner js-file-line">	vector&lt;<span class="pl-k">float</span>&gt; speedVector;</td>
      </tr>
      <tr>
        <td id="L1040" class="blob-num js-line-number" data-line-number="1040"></td>
        <td id="LC1040" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//if (initial_speed &gt; targetSpeed) return speedVector;</span></td>
      </tr>
      <tr>
        <td id="L1041" class="blob-num js-line-number" data-line-number="1041"></td>
        <td id="LC1041" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1042" class="blob-num js-line-number" data-line-number="1042"></td>
        <td id="LC1042" class="blob-code blob-code-inner js-file-line">	<span class="pl-c1">srand</span>(vehicle_id);</td>
      </tr>
      <tr>
        <td id="L1043" class="blob-num js-line-number" data-line-number="1043"></td>
        <td id="LC1043" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> maxDec = <span class="pl-c1">maxDecelerationBySpeed</span>(initial_speed);</td>
      </tr>
      <tr>
        <td id="L1044" class="blob-num js-line-number" data-line-number="1044"></td>
        <td id="LC1044" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> currDecRate = maxDec * (<span class="pl-k">float</span>)(<span class="pl-c1">rand</span>() / <span class="pl-c1">double</span>(RAND_MAX));</td>
      </tr>
      <tr>
        <td id="L1045" class="blob-num js-line-number" data-line-number="1045"></td>
        <td id="LC1045" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1046" class="blob-num js-line-number" data-line-number="1046"></td>
        <td id="LC1046" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">while</span> (speed + currDecRate &gt; targetSpeed)</td>
      </tr>
      <tr>
        <td id="L1047" class="blob-num js-line-number" data-line-number="1047"></td>
        <td id="LC1047" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L1048" class="blob-num js-line-number" data-line-number="1048"></td>
        <td id="LC1048" class="blob-code blob-code-inner js-file-line">		speed += currDecRate;</td>
      </tr>
      <tr>
        <td id="L1049" class="blob-num js-line-number" data-line-number="1049"></td>
        <td id="LC1049" class="blob-code blob-code-inner js-file-line">		speedVector.<span class="pl-c1">push_back</span>(speed);</td>
      </tr>
      <tr>
        <td id="L1050" class="blob-num js-line-number" data-line-number="1050"></td>
        <td id="LC1050" class="blob-code blob-code-inner js-file-line">		maxDec = <span class="pl-c1">maxDecelerationBySpeed</span>(initial_speed);</td>
      </tr>
      <tr>
        <td id="L1051" class="blob-num js-line-number" data-line-number="1051"></td>
        <td id="LC1051" class="blob-code blob-code-inner js-file-line">		currDecRate = maxDec * (<span class="pl-k">float</span>)(<span class="pl-c1">rand</span>() / <span class="pl-c1">double</span>(RAND_MAX));</td>
      </tr>
      <tr>
        <td id="L1052" class="blob-num js-line-number" data-line-number="1052"></td>
        <td id="LC1052" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1053" class="blob-num js-line-number" data-line-number="1053"></td>
        <td id="LC1053" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L1054" class="blob-num js-line-number" data-line-number="1054"></td>
        <td id="LC1054" class="blob-code blob-code-inner js-file-line">	speedVector.<span class="pl-c1">push_back</span>(targetSpeed);</td>
      </tr>
      <tr>
        <td id="L1055" class="blob-num js-line-number" data-line-number="1055"></td>
        <td id="LC1055" class="blob-code blob-code-inner js-file-line">	<span class="pl-c1">MovingAverage</span>(speedVector);</td>
      </tr>
      <tr>
        <td id="L1056" class="blob-num js-line-number" data-line-number="1056"></td>
        <td id="LC1056" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">return</span> speedVector;</td>
      </tr>
      <tr>
        <td id="L1057" class="blob-num js-line-number" data-line-number="1057"></td>
        <td id="LC1057" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L1058" class="blob-num js-line-number" data-line-number="1058"></td>
        <td id="LC1058" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1059" class="blob-num js-line-number" data-line-number="1059"></td>
        <td id="LC1059" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1060" class="blob-num js-line-number" data-line-number="1060"></td>
        <td id="LC1060" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1061" class="blob-num js-line-number" data-line-number="1061"></td>
        <td id="LC1061" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1062" class="blob-num js-line-number" data-line-number="1062"></td>
        <td id="LC1062" class="blob-code blob-code-inner js-file-line"><span class="pl-k">void</span> <span class="pl-en">SmoothVehicleTrajectory</span>(<span class="pl-k">int</span> vehicle_id, std::map&lt;<span class="pl-k">int</span>, VehicleSpeedProfileData&gt;&amp; speedProfile, <span class="pl-k">int</span> maxRun = <span class="pl-c1">11</span>)</td>
      </tr>
      <tr>
        <td id="L1063" class="blob-num js-line-number" data-line-number="1063"></td>
        <td id="LC1063" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L1064" class="blob-num js-line-number" data-line-number="1064"></td>
        <td id="LC1064" class="blob-code blob-code-inner js-file-line">	std::map&lt;<span class="pl-k">int</span>, VehicleSpeedProfileData&gt;::iterator iter = speedProfile.<span class="pl-c1">begin</span>();</td>
      </tr>
      <tr>
        <td id="L1065" class="blob-num js-line-number" data-line-number="1065"></td>
        <td id="LC1065" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> size = speedProfile.<span class="pl-c1">size</span>();</td>
      </tr>
      <tr>
        <td id="L1066" class="blob-num js-line-number" data-line-number="1066"></td>
        <td id="LC1066" class="blob-code blob-code-inner js-file-line">	vector&lt;<span class="pl-k">float</span>&gt; speedArray;</td>
      </tr>
      <tr>
        <td id="L1067" class="blob-num js-line-number" data-line-number="1067"></td>
        <td id="LC1067" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1068" class="blob-num js-line-number" data-line-number="1068"></td>
        <td id="LC1068" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//speedArray.push_back(0.0f);</span></td>
      </tr>
      <tr>
        <td id="L1069" class="blob-num js-line-number" data-line-number="1069"></td>
        <td id="LC1069" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">for</span> (<span class="pl-k">int</span> i=<span class="pl-c1">0</span>;iter != speedProfile.<span class="pl-c1">end</span>();iter++,i++)</td>
      </tr>
      <tr>
        <td id="L1070" class="blob-num js-line-number" data-line-number="1070"></td>
        <td id="LC1070" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L1071" class="blob-num js-line-number" data-line-number="1071"></td>
        <td id="LC1071" class="blob-code blob-code-inner js-file-line">		speedArray.<span class="pl-c1">push_back</span>(iter-&gt;second.<span class="pl-smi">Speed</span>);</td>
      </tr>
      <tr>
        <td id="L1072" class="blob-num js-line-number" data-line-number="1072"></td>
        <td id="LC1072" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L1073" class="blob-num js-line-number" data-line-number="1073"></td>
        <td id="LC1073" class="blob-code blob-code-inner js-file-line">	speedArray[speedArray.<span class="pl-c1">size</span>() - <span class="pl-c1">1</span>] = <span class="pl-c1">0</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L1074" class="blob-num js-line-number" data-line-number="1074"></td>
        <td id="LC1074" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1075" class="blob-num js-line-number" data-line-number="1075"></td>
        <td id="LC1075" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//Find the transition points</span></td>
      </tr>
      <tr>
        <td id="L1076" class="blob-num js-line-number" data-line-number="1076"></td>
        <td id="LC1076" class="blob-code blob-code-inner js-file-line">	vector&lt;TransitionPoint&gt; transitionPoints;</td>
      </tr>
      <tr>
        <td id="L1077" class="blob-num js-line-number" data-line-number="1077"></td>
        <td id="LC1077" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">for</span> (<span class="pl-k">int</span> i=<span class="pl-c1">1</span>;i&lt;size;i++)</td>
      </tr>
      <tr>
        <td id="L1078" class="blob-num js-line-number" data-line-number="1078"></td>
        <td id="LC1078" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L1079" class="blob-num js-line-number" data-line-number="1079"></td>
        <td id="LC1079" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">if</span> (speedArray[i] - speedArray[i-<span class="pl-c1">1</span>] &gt; <span class="pl-c1">5.6</span> || speedArray[i] - speedArray[i-<span class="pl-c1">1</span>] &lt; -<span class="pl-c1">5.1</span>)</td>
      </tr>
      <tr>
        <td id="L1080" class="blob-num js-line-number" data-line-number="1080"></td>
        <td id="LC1080" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L1081" class="blob-num js-line-number" data-line-number="1081"></td>
        <td id="LC1081" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (i != size -<span class="pl-c1">1</span>)</td>
      </tr>
      <tr>
        <td id="L1082" class="blob-num js-line-number" data-line-number="1082"></td>
        <td id="LC1082" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L1083" class="blob-num js-line-number" data-line-number="1083"></td>
        <td id="LC1083" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (<span class="pl-c1">abs</span>(speedArray[i] - speedArray[i-<span class="pl-c1">1</span>]) == <span class="pl-c1">abs</span>(speedArray[i] - speedArray[i+<span class="pl-c1">1</span>]))</td>
      </tr>
      <tr>
        <td id="L1084" class="blob-num js-line-number" data-line-number="1084"></td>
        <td id="LC1084" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L1085" class="blob-num js-line-number" data-line-number="1085"></td>
        <td id="LC1085" class="blob-code blob-code-inner js-file-line">					speedArray[i] = speedArray[i-<span class="pl-c1">1</span>];</td>
      </tr>
      <tr>
        <td id="L1086" class="blob-num js-line-number" data-line-number="1086"></td>
        <td id="LC1086" class="blob-code blob-code-inner js-file-line">					i++;</td>
      </tr>
      <tr>
        <td id="L1087" class="blob-num js-line-number" data-line-number="1087"></td>
        <td id="LC1087" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">continue</span>;</td>
      </tr>
      <tr>
        <td id="L1088" class="blob-num js-line-number" data-line-number="1088"></td>
        <td id="LC1088" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L1089" class="blob-num js-line-number" data-line-number="1089"></td>
        <td id="LC1089" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L1090" class="blob-num js-line-number" data-line-number="1090"></td>
        <td id="LC1090" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1091" class="blob-num js-line-number" data-line-number="1091"></td>
        <td id="LC1091" class="blob-code blob-code-inner js-file-line">			TransitionPoint point;</td>
      </tr>
      <tr>
        <td id="L1092" class="blob-num js-line-number" data-line-number="1092"></td>
        <td id="LC1092" class="blob-code blob-code-inner js-file-line">			point.<span class="pl-smi">t</span> = i;</td>
      </tr>
      <tr>
        <td id="L1093" class="blob-num js-line-number" data-line-number="1093"></td>
        <td id="LC1093" class="blob-code blob-code-inner js-file-line">			point.<span class="pl-smi">type</span> = speedArray[i] - speedArray[i-<span class="pl-c1">1</span>] &gt; <span class="pl-c1">0</span> ? ACC : DEC;</td>
      </tr>
      <tr>
        <td id="L1094" class="blob-num js-line-number" data-line-number="1094"></td>
        <td id="LC1094" class="blob-code blob-code-inner js-file-line">			transitionPoints.<span class="pl-c1">push_back</span>(point);</td>
      </tr>
      <tr>
        <td id="L1095" class="blob-num js-line-number" data-line-number="1095"></td>
        <td id="LC1095" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L1096" class="blob-num js-line-number" data-line-number="1096"></td>
        <td id="LC1096" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L1097" class="blob-num js-line-number" data-line-number="1097"></td>
        <td id="LC1097" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1098" class="blob-num js-line-number" data-line-number="1098"></td>
        <td id="LC1098" class="blob-code blob-code-inner js-file-line">	<span class="pl-c1">MovingAverage</span>(speedArray);</td>
      </tr>
      <tr>
        <td id="L1099" class="blob-num js-line-number" data-line-number="1099"></td>
        <td id="LC1099" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1100" class="blob-num js-line-number" data-line-number="1100"></td>
        <td id="LC1100" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (transitionPoints.<span class="pl-c1">size</span>() &lt;= <span class="pl-c1">0</span>) <span class="pl-k">return</span>;</td>
      </tr>
      <tr>
        <td id="L1101" class="blob-num js-line-number" data-line-number="1101"></td>
        <td id="LC1101" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1102" class="blob-num js-line-number" data-line-number="1102"></td>
        <td id="LC1102" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//Do the smoothing based on accelertion or deceleration</span></td>
      </tr>
      <tr>
        <td id="L1103" class="blob-num js-line-number" data-line-number="1103"></td>
        <td id="LC1103" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">for</span> (<span class="pl-k">int</span> i=<span class="pl-c1">0</span>;i&lt;transitionPoints.<span class="pl-c1">size</span>();i++)</td>
      </tr>
      <tr>
        <td id="L1104" class="blob-num js-line-number" data-line-number="1104"></td>
        <td id="LC1104" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L1105" class="blob-num js-line-number" data-line-number="1105"></td>
        <td id="LC1105" class="blob-code blob-code-inner js-file-line">		TransitionPoint currPoint = transitionPoints[i];</td>
      </tr>
      <tr>
        <td id="L1106" class="blob-num js-line-number" data-line-number="1106"></td>
        <td id="LC1106" class="blob-code blob-code-inner js-file-line">		<span class="pl-c">//if (i &gt; 0)</span></td>
      </tr>
      <tr>
        <td id="L1107" class="blob-num js-line-number" data-line-number="1107"></td>
        <td id="LC1107" class="blob-code blob-code-inner js-file-line">		<span class="pl-c">//{</span></td>
      </tr>
      <tr>
        <td id="L1108" class="blob-num js-line-number" data-line-number="1108"></td>
        <td id="LC1108" class="blob-code blob-code-inner js-file-line">		<span class="pl-c">//	TransitionPoint prevPoint = transitionPoints[i-1];</span></td>
      </tr>
      <tr>
        <td id="L1109" class="blob-num js-line-number" data-line-number="1109"></td>
        <td id="LC1109" class="blob-code blob-code-inner js-file-line">		<span class="pl-c">//	if (currPoint.t - prevPoint.t &lt; 5)</span></td>
      </tr>
      <tr>
        <td id="L1110" class="blob-num js-line-number" data-line-number="1110"></td>
        <td id="LC1110" class="blob-code blob-code-inner js-file-line">		<span class="pl-c">//	{</span></td>
      </tr>
      <tr>
        <td id="L1111" class="blob-num js-line-number" data-line-number="1111"></td>
        <td id="LC1111" class="blob-code blob-code-inner js-file-line">		<span class="pl-c">//		continue;</span></td>
      </tr>
      <tr>
        <td id="L1112" class="blob-num js-line-number" data-line-number="1112"></td>
        <td id="LC1112" class="blob-code blob-code-inner js-file-line">		<span class="pl-c">//	}</span></td>
      </tr>
      <tr>
        <td id="L1113" class="blob-num js-line-number" data-line-number="1113"></td>
        <td id="LC1113" class="blob-code blob-code-inner js-file-line">		<span class="pl-c">//}</span></td>
      </tr>
      <tr>
        <td id="L1114" class="blob-num js-line-number" data-line-number="1114"></td>
        <td id="LC1114" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">float</span> initialSpeed = speedArray[currPoint.<span class="pl-smi">t</span> -<span class="pl-c1">1</span>];</td>
      </tr>
      <tr>
        <td id="L1115" class="blob-num js-line-number" data-line-number="1115"></td>
        <td id="LC1115" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">float</span> targetSpeed = speedArray[currPoint.<span class="pl-smi">t</span>];</td>
      </tr>
      <tr>
        <td id="L1116" class="blob-num js-line-number" data-line-number="1116"></td>
        <td id="LC1116" class="blob-code blob-code-inner js-file-line">		vector&lt;<span class="pl-k">float</span>&gt; speedVector;</td>
      </tr>
      <tr>
        <td id="L1117" class="blob-num js-line-number" data-line-number="1117"></td>
        <td id="LC1117" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">if</span> (currPoint.<span class="pl-smi">type</span> == ACC)</td>
      </tr>
      <tr>
        <td id="L1118" class="blob-num js-line-number" data-line-number="1118"></td>
        <td id="LC1118" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L1119" class="blob-num js-line-number" data-line-number="1119"></td>
        <td id="LC1119" class="blob-code blob-code-inner js-file-line">			speedVector = <span class="pl-c1">SpeedUpTo</span>(vehicle_id, initialSpeed, targetSpeed);</td>
      </tr>
      <tr>
        <td id="L1120" class="blob-num js-line-number" data-line-number="1120"></td>
        <td id="LC1120" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (i != transitionPoints.<span class="pl-c1">size</span>() - <span class="pl-c1">1</span>)</td>
      </tr>
      <tr>
        <td id="L1121" class="blob-num js-line-number" data-line-number="1121"></td>
        <td id="LC1121" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L1122" class="blob-num js-line-number" data-line-number="1122"></td>
        <td id="LC1122" class="blob-code blob-code-inner js-file-line">				TransitionPoint nextPoint = transitionPoints[i+<span class="pl-c1">1</span>];</td>
      </tr>
      <tr>
        <td id="L1123" class="blob-num js-line-number" data-line-number="1123"></td>
        <td id="LC1123" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1124" class="blob-num js-line-number" data-line-number="1124"></td>
        <td id="LC1124" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (speedVector.<span class="pl-c1">size</span>() &gt; <span class="pl-c1">0</span> <span class="pl-c">/*&amp;&amp; nextPoint.t - currPoint.t &gt; speedVector.size()*/</span>)</td>
      </tr>
      <tr>
        <td id="L1125" class="blob-num js-line-number" data-line-number="1125"></td>
        <td id="LC1125" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L1126" class="blob-num js-line-number" data-line-number="1126"></td>
        <td id="LC1126" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">if</span> (currPoint.<span class="pl-smi">t</span> &gt;= speedVector.<span class="pl-c1">size</span>())</td>
      </tr>
      <tr>
        <td id="L1127" class="blob-num js-line-number" data-line-number="1127"></td>
        <td id="LC1127" class="blob-code blob-code-inner js-file-line">					{</td>
      </tr>
      <tr>
        <td id="L1128" class="blob-num js-line-number" data-line-number="1128"></td>
        <td id="LC1128" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">for</span> (<span class="pl-k">int</span> n = <span class="pl-c1">0</span>; n &lt; speedVector.<span class="pl-c1">size</span>(); n++)</td>
      </tr>
      <tr>
        <td id="L1129" class="blob-num js-line-number" data-line-number="1129"></td>
        <td id="LC1129" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L1130" class="blob-num js-line-number" data-line-number="1130"></td>
        <td id="LC1130" class="blob-code blob-code-inner js-file-line">							speedArray[currPoint.<span class="pl-smi">t</span> - speedVector.<span class="pl-c1">size</span>() + <span class="pl-c1">1</span> + n] = speedVector[n];</td>
      </tr>
      <tr>
        <td id="L1131" class="blob-num js-line-number" data-line-number="1131"></td>
        <td id="LC1131" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L1132" class="blob-num js-line-number" data-line-number="1132"></td>
        <td id="LC1132" class="blob-code blob-code-inner js-file-line">					}</td>
      </tr>
      <tr>
        <td id="L1133" class="blob-num js-line-number" data-line-number="1133"></td>
        <td id="LC1133" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L1134" class="blob-num js-line-number" data-line-number="1134"></td>
        <td id="LC1134" class="blob-code blob-code-inner js-file-line">					{</td>
      </tr>
      <tr>
        <td id="L1135" class="blob-num js-line-number" data-line-number="1135"></td>
        <td id="LC1135" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">for</span> (<span class="pl-k">int</span> n = <span class="pl-c1">0</span>; n &lt; speedVector.<span class="pl-c1">size</span>(); n++)</td>
      </tr>
      <tr>
        <td id="L1136" class="blob-num js-line-number" data-line-number="1136"></td>
        <td id="LC1136" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L1137" class="blob-num js-line-number" data-line-number="1137"></td>
        <td id="LC1137" class="blob-code blob-code-inner js-file-line">							speedArray[currPoint.<span class="pl-smi">t</span> + n] = speedVector[n];</td>
      </tr>
      <tr>
        <td id="L1138" class="blob-num js-line-number" data-line-number="1138"></td>
        <td id="LC1138" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L1139" class="blob-num js-line-number" data-line-number="1139"></td>
        <td id="LC1139" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1140" class="blob-num js-line-number" data-line-number="1140"></td>
        <td id="LC1140" class="blob-code blob-code-inner js-file-line">					}</td>
      </tr>
      <tr>
        <td id="L1141" class="blob-num js-line-number" data-line-number="1141"></td>
        <td id="LC1141" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L1142" class="blob-num js-line-number" data-line-number="1142"></td>
        <td id="LC1142" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L1143" class="blob-num js-line-number" data-line-number="1143"></td>
        <td id="LC1143" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L1144" class="blob-num js-line-number" data-line-number="1144"></td>
        <td id="LC1144" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L1145" class="blob-num js-line-number" data-line-number="1145"></td>
        <td id="LC1145" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L1146" class="blob-num js-line-number" data-line-number="1146"></td>
        <td id="LC1146" class="blob-code blob-code-inner js-file-line">			speedVector = <span class="pl-c1">SlowDownTo</span>(vehicle_id, initialSpeed, targetSpeed);</td>
      </tr>
      <tr>
        <td id="L1147" class="blob-num js-line-number" data-line-number="1147"></td>
        <td id="LC1147" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (speedVector.<span class="pl-c1">size</span>() &gt; <span class="pl-c1">0</span> &amp;&amp; currPoint.<span class="pl-smi">t</span> &gt;= speedVector.<span class="pl-c1">size</span>())</td>
      </tr>
      <tr>
        <td id="L1148" class="blob-num js-line-number" data-line-number="1148"></td>
        <td id="LC1148" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L1149" class="blob-num js-line-number" data-line-number="1149"></td>
        <td id="LC1149" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">for</span> (<span class="pl-k">int</span> n = <span class="pl-c1">0</span>; n &lt; speedVector.<span class="pl-c1">size</span>(); n++)</td>
      </tr>
      <tr>
        <td id="L1150" class="blob-num js-line-number" data-line-number="1150"></td>
        <td id="LC1150" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L1151" class="blob-num js-line-number" data-line-number="1151"></td>
        <td id="LC1151" class="blob-code blob-code-inner js-file-line">					speedArray[currPoint.<span class="pl-smi">t</span> - speedVector.<span class="pl-c1">size</span>() + <span class="pl-c1">1</span> + n] = speedVector[n];</td>
      </tr>
      <tr>
        <td id="L1152" class="blob-num js-line-number" data-line-number="1152"></td>
        <td id="LC1152" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L1153" class="blob-num js-line-number" data-line-number="1153"></td>
        <td id="LC1153" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L1154" class="blob-num js-line-number" data-line-number="1154"></td>
        <td id="LC1154" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L1155" class="blob-num js-line-number" data-line-number="1155"></td>
        <td id="LC1155" class="blob-code blob-code-inner js-file-line">		</td>
      </tr>
      <tr>
        <td id="L1156" class="blob-num js-line-number" data-line-number="1156"></td>
        <td id="LC1156" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L1157" class="blob-num js-line-number" data-line-number="1157"></td>
        <td id="LC1157" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1158" class="blob-num js-line-number" data-line-number="1158"></td>
        <td id="LC1158" class="blob-code blob-code-inner js-file-line">	<span class="pl-c1">MovingAverage</span>(speedArray);</td>
      </tr>
      <tr>
        <td id="L1159" class="blob-num js-line-number" data-line-number="1159"></td>
        <td id="LC1159" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1160" class="blob-num js-line-number" data-line-number="1160"></td>
        <td id="LC1160" class="blob-code blob-code-inner js-file-line">	iter = speedProfile.<span class="pl-c1">begin</span>();</td>
      </tr>
      <tr>
        <td id="L1161" class="blob-num js-line-number" data-line-number="1161"></td>
        <td id="LC1161" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1162" class="blob-num js-line-number" data-line-number="1162"></td>
        <td id="LC1162" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1163" class="blob-num js-line-number" data-line-number="1163"></td>
        <td id="LC1163" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">for</span> (<span class="pl-k">int</span> i=<span class="pl-c1">0</span>;iter != speedProfile.<span class="pl-c1">end</span>();iter++,i++)</td>
      </tr>
      <tr>
        <td id="L1164" class="blob-num js-line-number" data-line-number="1164"></td>
        <td id="LC1164" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L1165" class="blob-num js-line-number" data-line-number="1165"></td>
        <td id="LC1165" class="blob-code blob-code-inner js-file-line">		iter-&gt;second.<span class="pl-smi">Speed</span> = speedArray[i];</td>
      </tr>
      <tr>
        <td id="L1166" class="blob-num js-line-number" data-line-number="1166"></td>
        <td id="LC1166" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">if</span> (i == <span class="pl-c1">0</span>)</td>
      </tr>
      <tr>
        <td id="L1167" class="blob-num js-line-number" data-line-number="1167"></td>
        <td id="LC1167" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L1168" class="blob-num js-line-number" data-line-number="1168"></td>
        <td id="LC1168" class="blob-code blob-code-inner js-file-line">			iter-&gt;second.<span class="pl-smi">Acceleration</span> = <span class="pl-c1">0</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L1169" class="blob-num js-line-number" data-line-number="1169"></td>
        <td id="LC1169" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L1170" class="blob-num js-line-number" data-line-number="1170"></td>
        <td id="LC1170" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L1171" class="blob-num js-line-number" data-line-number="1171"></td>
        <td id="LC1171" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L1172" class="blob-num js-line-number" data-line-number="1172"></td>
        <td id="LC1172" class="blob-code blob-code-inner js-file-line">			iter-&gt;second.<span class="pl-smi">Acceleration</span> = speedArray[i] - speedArray[i - <span class="pl-c1">1</span>];</td>
      </tr>
      <tr>
        <td id="L1173" class="blob-num js-line-number" data-line-number="1173"></td>
        <td id="LC1173" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">//if (iter-&gt;second.Acceleration &gt; 0 &amp;&amp; iter-&gt;second.Acceleration </span></td>
      </tr>
      <tr>
        <td id="L1174" class="blob-num js-line-number" data-line-number="1174"></td>
        <td id="LC1174" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L1175" class="blob-num js-line-number" data-line-number="1175"></td>
        <td id="LC1175" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L1176" class="blob-num js-line-number" data-line-number="1176"></td>
        <td id="LC1176" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1177" class="blob-num js-line-number" data-line-number="1177"></td>
        <td id="LC1177" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (maxRun &gt; <span class="pl-c1">0</span>)</td>
      </tr>
      <tr>
        <td id="L1178" class="blob-num js-line-number" data-line-number="1178"></td>
        <td id="LC1178" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L1179" class="blob-num js-line-number" data-line-number="1179"></td>
        <td id="LC1179" class="blob-code blob-code-inner js-file-line">		<span class="pl-c1">SmoothVehicleTrajectory</span>(vehicle_id, speedProfile, --maxRun);</td>
      </tr>
      <tr>
        <td id="L1180" class="blob-num js-line-number" data-line-number="1180"></td>
        <td id="LC1180" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L1181" class="blob-num js-line-number" data-line-number="1181"></td>
        <td id="LC1181" class="blob-code blob-code-inner js-file-line">		</td>
      </tr>
      <tr>
        <td id="L1182" class="blob-num js-line-number" data-line-number="1182"></td>
        <td id="LC1182" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L1183" class="blob-num js-line-number" data-line-number="1183"></td>
        <td id="LC1183" class="blob-code blob-code-inner js-file-line"><span class="pl-k">void</span> <span class="pl-en">g_CalculateEmissionMOE</span>()</td>
      </tr>
      <tr>
        <td id="L1184" class="blob-num js-line-number" data-line-number="1184"></td>
        <td id="LC1184" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L1185" class="blob-num js-line-number" data-line-number="1185"></td>
        <td id="LC1185" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">ifdef</span> _large_memory_usage</td>
      </tr>
      <tr>
        <td id="L1186" class="blob-num js-line-number" data-line-number="1186"></td>
        <td id="LC1186" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1187" class="blob-num js-line-number" data-line-number="1187"></td>
        <td id="LC1187" class="blob-code blob-code-inner js-file-line">	std::set&lt;DTALink*&gt;::iterator iterLink;</td>
      </tr>
      <tr>
        <td id="L1188" class="blob-num js-line-number" data-line-number="1188"></td>
        <td id="LC1188" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> count = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L1189" class="blob-num js-line-number" data-line-number="1189"></td>
        <td id="LC1189" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">// step 1: generate data for different lanes</span></td>
      </tr>
      <tr>
        <td id="L1190" class="blob-num js-line-number" data-line-number="1190"></td>
        <td id="LC1190" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">for</span> (<span class="pl-k">unsigned</span> li = <span class="pl-c1">0</span>; li&lt; g_LinkVector.<span class="pl-c1">size</span>(); li++)</td>
      </tr>
      <tr>
        <td id="L1191" class="blob-num js-line-number" data-line-number="1191"></td>
        <td id="LC1191" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L1192" class="blob-num js-line-number" data-line-number="1192"></td>
        <td id="LC1192" class="blob-code blob-code-inner js-file-line">		DTALink* pLink = g_LinkVector[li];</td>
      </tr>
      <tr>
        <td id="L1193" class="blob-num js-line-number" data-line-number="1193"></td>
        <td id="LC1193" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1194" class="blob-num js-line-number" data-line-number="1194"></td>
        <td id="LC1194" class="blob-code blob-code-inner js-file-line">		pLink-&gt;m_VehicleDataVector.<span class="pl-c1">clear</span>();</td>
      </tr>
      <tr>
        <td id="L1195" class="blob-num js-line-number" data-line-number="1195"></td>
        <td id="LC1195" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">for</span> (<span class="pl-k">int</span> LaneNo = <span class="pl-c1">0</span>; LaneNo &lt; pLink-&gt;m_OutflowNumLanes; LaneNo++)</td>
      </tr>
      <tr>
        <td id="L1196" class="blob-num js-line-number" data-line-number="1196"></td>
        <td id="LC1196" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L1197" class="blob-num js-line-number" data-line-number="1197"></td>
        <td id="LC1197" class="blob-code blob-code-inner js-file-line">			LaneVehicleCFData <span class="pl-smi">element</span>(g_PlanningHorizon + <span class="pl-c1">1</span>);</td>
      </tr>
      <tr>
        <td id="L1198" class="blob-num js-line-number" data-line-number="1198"></td>
        <td id="LC1198" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1199" class="blob-num js-line-number" data-line-number="1199"></td>
        <td id="LC1199" class="blob-code blob-code-inner js-file-line">			pLink-&gt;m_VehicleDataVector.<span class="pl-c1">push_back</span>(element);</td>
      </tr>
      <tr>
        <td id="L1200" class="blob-num js-line-number" data-line-number="1200"></td>
        <td id="LC1200" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L1201" class="blob-num js-line-number" data-line-number="1201"></td>
        <td id="LC1201" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L1202" class="blob-num js-line-number" data-line-number="1202"></td>
        <td id="LC1202" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1203" class="blob-num js-line-number" data-line-number="1203"></td>
        <td id="LC1203" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">// step 2: collect all vehicles passing each link</span></td>
      </tr>
      <tr>
        <td id="L1204" class="blob-num js-line-number" data-line-number="1204"></td>
        <td id="LC1204" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1205" class="blob-num js-line-number" data-line-number="1205"></td>
        <td id="LC1205" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> totalSamples = (<span class="pl-k">int</span>)(g_VehicleMap.<span class="pl-c1">size</span>() * g_OutputSecondBySecondEmissionDataPercentage);</td>
      </tr>
      <tr>
        <td id="L1206" class="blob-num js-line-number" data-line-number="1206"></td>
        <td id="LC1206" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> sampling_interval = (<span class="pl-k">int</span>)(g_VehicleMap.<span class="pl-c1">size</span>() / <span class="pl-c1">max</span>(<span class="pl-c1">1</span>, totalSamples));</td>
      </tr>
      <tr>
        <td id="L1207" class="blob-num js-line-number" data-line-number="1207"></td>
        <td id="LC1207" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> vehicleCounts = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L1208" class="blob-num js-line-number" data-line-number="1208"></td>
        <td id="LC1208" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1209" class="blob-num js-line-number" data-line-number="1209"></td>
        <td id="LC1209" class="blob-code blob-code-inner js-file-line">	std::map&lt;<span class="pl-k">int</span>, DTAVehicle*&gt;::iterator iterVM;</td>
      </tr>
      <tr>
        <td id="L1210" class="blob-num js-line-number" data-line-number="1210"></td>
        <td id="LC1210" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">for</span> (iterVM = g_VehicleMap.<span class="pl-c1">begin</span>(); iterVM != g_VehicleMap.<span class="pl-c1">end</span>(); iterVM++)</td>
      </tr>
      <tr>
        <td id="L1211" class="blob-num js-line-number" data-line-number="1211"></td>
        <td id="LC1211" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L1212" class="blob-num js-line-number" data-line-number="1212"></td>
        <td id="LC1212" class="blob-code blob-code-inner js-file-line">		DTAVehicle* pVehicle = iterVM-&gt;second;</td>
      </tr>
      <tr>
        <td id="L1213" class="blob-num js-line-number" data-line-number="1213"></td>
        <td id="LC1213" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1214" class="blob-num js-line-number" data-line-number="1214"></td>
        <td id="LC1214" class="blob-code blob-code-inner js-file-line">		pVehicle-&gt;m_OperatingModeCount.<span class="pl-c1">clear</span>();</td>
      </tr>
      <tr>
        <td id="L1215" class="blob-num js-line-number" data-line-number="1215"></td>
        <td id="LC1215" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1216" class="blob-num js-line-number" data-line-number="1216"></td>
        <td id="LC1216" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">if</span> (pVehicle-&gt;m_NodeSize &gt;= <span class="pl-c1">2</span>)</td>
      </tr>
      <tr>
        <td id="L1217" class="blob-num js-line-number" data-line-number="1217"></td>
        <td id="LC1217" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L1218" class="blob-num js-line-number" data-line-number="1218"></td>
        <td id="LC1218" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">for</span> (<span class="pl-k">int</span> i = <span class="pl-c1">0</span>; i&lt; pVehicle-&gt;m_NodeSize - <span class="pl-c1">1</span>; i++)</td>
      </tr>
      <tr>
        <td id="L1219" class="blob-num js-line-number" data-line-number="1219"></td>
        <td id="LC1219" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L1220" class="blob-num js-line-number" data-line-number="1220"></td>
        <td id="LC1220" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (pVehicle-&gt;m_bComplete)  <span class="pl-c">// for vehicles finish the trips</span></td>
      </tr>
      <tr>
        <td id="L1221" class="blob-num js-line-number" data-line-number="1221"></td>
        <td id="LC1221" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L1222" class="blob-num js-line-number" data-line-number="1222"></td>
        <td id="LC1222" class="blob-code blob-code-inner js-file-line">					VehicleCFData element;</td>
      </tr>
      <tr>
        <td id="L1223" class="blob-num js-line-number" data-line-number="1223"></td>
        <td id="LC1223" class="blob-code blob-code-inner js-file-line">					element.<span class="pl-smi">VehicleID</span> = pVehicle-&gt;m_AgentID;</td>
      </tr>
      <tr>
        <td id="L1224" class="blob-num js-line-number" data-line-number="1224"></td>
        <td id="LC1224" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">int</span> LinkNo = pVehicle-&gt;m_LinkAry[i].<span class="pl-smi">LinkNo</span>;</td>
      </tr>
      <tr>
        <td id="L1225" class="blob-num js-line-number" data-line-number="1225"></td>
        <td id="LC1225" class="blob-code blob-code-inner js-file-line">					element.<span class="pl-smi">SequentialLinkNo</span> = i;</td>
      </tr>
      <tr>
        <td id="L1226" class="blob-num js-line-number" data-line-number="1226"></td>
        <td id="LC1226" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1227" class="blob-num js-line-number" data-line-number="1227"></td>
        <td id="LC1227" class="blob-code blob-code-inner js-file-line">					element.<span class="pl-smi">FreeflowDistance_per_SimulationInterval</span> = g_LinkVector[LinkNo]-&gt;m_SpeedLimit* <span class="pl-c1">1609</span>.<span class="pl-c1">344f</span> / <span class="pl-c1">3600</span> / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;</td>
      </tr>
      <tr>
        <td id="L1228" class="blob-num js-line-number" data-line-number="1228"></td>
        <td id="LC1228" class="blob-code blob-code-inner js-file-line">					<span class="pl-c">//1609.344f: mile to meters ; 3600: # of seconds per hour</span></td>
      </tr>
      <tr>
        <td id="L1229" class="blob-num js-line-number" data-line-number="1229"></td>
        <td id="LC1229" class="blob-code blob-code-inner js-file-line">					<span class="pl-c">//τ = 1/(wkj)</span></td>
      </tr>
      <tr>
        <td id="L1230" class="blob-num js-line-number" data-line-number="1230"></td>
        <td id="LC1230" class="blob-code blob-code-inner js-file-line">					<span class="pl-c">// δ = 1/kj</span></td>
      </tr>
      <tr>
        <td id="L1231" class="blob-num js-line-number" data-line-number="1231"></td>
        <td id="LC1231" class="blob-code blob-code-inner js-file-line">					element.<span class="pl-smi">TimeLag_in_SimulationInterval</span> = (<span class="pl-k">int</span>)(<span class="pl-c1">3600</span> * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND / (g_LinkVector[LinkNo]-&gt;m_BackwardWaveSpeed * g_LinkVector[LinkNo]-&gt;m_KJam) + <span class="pl-c1">0.5</span>);</td>
      </tr>
      <tr>
        <td id="L1232" class="blob-num js-line-number" data-line-number="1232"></td>
        <td id="LC1232" class="blob-code blob-code-inner js-file-line">					element.<span class="pl-smi">CriticalSpacing_in_meter</span> = <span class="pl-c1">1609</span>.<span class="pl-c1">344f</span> / g_LinkVector[LinkNo]-&gt;m_KJam;</td>
      </tr>
      <tr>
        <td id="L1233" class="blob-num js-line-number" data-line-number="1233"></td>
        <td id="LC1233" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1234" class="blob-num js-line-number" data-line-number="1234"></td>
        <td id="LC1234" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">if</span> (i == <span class="pl-c1">0</span>)</td>
      </tr>
      <tr>
        <td id="L1235" class="blob-num js-line-number" data-line-number="1235"></td>
        <td id="LC1235" class="blob-code blob-code-inner js-file-line">					{</td>
      </tr>
      <tr>
        <td id="L1236" class="blob-num js-line-number" data-line-number="1236"></td>
        <td id="LC1236" class="blob-code blob-code-inner js-file-line">						element.<span class="pl-smi">StartTime_in_SimulationInterval</span> = pVehicle-&gt;m_DepartureTime * <span class="pl-c1">60</span> * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;</td>
      </tr>
      <tr>
        <td id="L1237" class="blob-num js-line-number" data-line-number="1237"></td>
        <td id="LC1237" class="blob-code blob-code-inner js-file-line">					}</td>
      </tr>
      <tr>
        <td id="L1238" class="blob-num js-line-number" data-line-number="1238"></td>
        <td id="LC1238" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L1239" class="blob-num js-line-number" data-line-number="1239"></td>
        <td id="LC1239" class="blob-code blob-code-inner js-file-line">					{</td>
      </tr>
      <tr>
        <td id="L1240" class="blob-num js-line-number" data-line-number="1240"></td>
        <td id="LC1240" class="blob-code blob-code-inner js-file-line">						element.<span class="pl-smi">StartTime_in_SimulationInterval</span> = pVehicle-&gt;m_LinkAry[i - <span class="pl-c1">1</span>].<span class="pl-smi">AbsArrivalTimeOnDSN</span> * <span class="pl-c1">60</span> * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;</td>
      </tr>
      <tr>
        <td id="L1241" class="blob-num js-line-number" data-line-number="1241"></td>
        <td id="LC1241" class="blob-code blob-code-inner js-file-line">					}</td>
      </tr>
      <tr>
        <td id="L1242" class="blob-num js-line-number" data-line-number="1242"></td>
        <td id="LC1242" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1243" class="blob-num js-line-number" data-line-number="1243"></td>
        <td id="LC1243" class="blob-code blob-code-inner js-file-line">					element.<span class="pl-smi">EndTime_in_SimulationInterval</span> = pVehicle-&gt;m_LinkAry[i].<span class="pl-smi">AbsArrivalTimeOnDSN</span> * <span class="pl-c1">60</span> * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;</td>
      </tr>
      <tr>
        <td id="L1244" class="blob-num js-line-number" data-line-number="1244"></td>
        <td id="LC1244" class="blob-code blob-code-inner js-file-line">					element.<span class="pl-smi">LaneNo</span> = element.<span class="pl-smi">VehicleID</span>%g_LinkVector[LinkNo]-&gt;m_OutflowNumLanes;</td>
      </tr>
      <tr>
        <td id="L1245" class="blob-num js-line-number" data-line-number="1245"></td>
        <td id="LC1245" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1246" class="blob-num js-line-number" data-line-number="1246"></td>
        <td id="LC1246" class="blob-code blob-code-inner js-file-line">					g_LinkVector[LinkNo]-&gt;m_VehicleDataVector[element.<span class="pl-smi">LaneNo</span>].<span class="pl-smi">LaneData</span>.<span class="pl-c1">push_back</span>(element);</td>
      </tr>
      <tr>
        <td id="L1247" class="blob-num js-line-number" data-line-number="1247"></td>
        <td id="LC1247" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L1248" class="blob-num js-line-number" data-line-number="1248"></td>
        <td id="LC1248" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L1249" class="blob-num js-line-number" data-line-number="1249"></td>
        <td id="LC1249" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1250" class="blob-num js-line-number" data-line-number="1250"></td>
        <td id="LC1250" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (pVehicle-&gt;m_bComplete)</td>
      </tr>
      <tr>
        <td id="L1251" class="blob-num js-line-number" data-line-number="1251"></td>
        <td id="LC1251" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L1252" class="blob-num js-line-number" data-line-number="1252"></td>
        <td id="LC1252" class="blob-code blob-code-inner js-file-line">				vehicleCounts++;</td>
      </tr>
      <tr>
        <td id="L1253" class="blob-num js-line-number" data-line-number="1253"></td>
        <td id="LC1253" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (vehicleCounts % sampling_interval == <span class="pl-c1">0</span>)</td>
      </tr>
      <tr>
        <td id="L1254" class="blob-num js-line-number" data-line-number="1254"></td>
        <td id="LC1254" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L1255" class="blob-num js-line-number" data-line-number="1255"></td>
        <td id="LC1255" class="blob-code blob-code-inner js-file-line">					pVehicle-&gt;m_bDetailedEmissionOutput = <span class="pl-c1">true</span>;</td>
      </tr>
      <tr>
        <td id="L1256" class="blob-num js-line-number" data-line-number="1256"></td>
        <td id="LC1256" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L1257" class="blob-num js-line-number" data-line-number="1257"></td>
        <td id="LC1257" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L1258" class="blob-num js-line-number" data-line-number="1258"></td>
        <td id="LC1258" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">// for each vehicle</span></td>
      </tr>
      <tr>
        <td id="L1259" class="blob-num js-line-number" data-line-number="1259"></td>
        <td id="LC1259" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L1260" class="blob-num js-line-number" data-line-number="1260"></td>
        <td id="LC1260" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L1261" class="blob-num js-line-number" data-line-number="1261"></td>
        <td id="LC1261" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1262" class="blob-num js-line-number" data-line-number="1262"></td>
        <td id="LC1262" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1263" class="blob-num js-line-number" data-line-number="1263"></td>
        <td id="LC1263" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">// calclate link based VSP</span></td>
      </tr>
      <tr>
        <td id="L1264" class="blob-num js-line-number" data-line-number="1264"></td>
        <td id="LC1264" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">// step 3: collect all vehicle passing each link</span></td>
      </tr>
      <tr>
        <td id="L1265" class="blob-num js-line-number" data-line-number="1265"></td>
        <td id="LC1265" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">for</span> (<span class="pl-k">int</span> li = <span class="pl-c1">0</span>; li&lt; g_LinkVector.<span class="pl-c1">size</span>(); li++)</td>
      </tr>
      <tr>
        <td id="L1266" class="blob-num js-line-number" data-line-number="1266"></td>
        <td id="LC1266" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L1267" class="blob-num js-line-number" data-line-number="1267"></td>
        <td id="LC1267" class="blob-code blob-code-inner js-file-line">		<span class="pl-c1">TRACE</span>(<span class="pl-s"><span class="pl-pds">&quot;</span><span class="pl-cce">\n</span> compute emissions for link <span class="pl-c1">%d</span><span class="pl-pds">&quot;</span></span>, li);</td>
      </tr>
      <tr>
        <td id="L1268" class="blob-num js-line-number" data-line-number="1268"></td>
        <td id="LC1268" class="blob-code blob-code-inner js-file-line">		<span class="pl-c">//		g_LogFile &lt;&lt; &quot;compute emissions for link &quot; &lt;&lt; li &lt;&lt; endl;</span></td>
      </tr>
      <tr>
        <td id="L1269" class="blob-num js-line-number" data-line-number="1269"></td>
        <td id="LC1269" class="blob-code blob-code-inner js-file-line">		cout &lt;&lt; <span class="pl-s"><span class="pl-pds">&quot;</span>compute emissions for link <span class="pl-pds">&quot;</span></span> &lt;&lt; li &lt;&lt; endl;</td>
      </tr>
      <tr>
        <td id="L1270" class="blob-num js-line-number" data-line-number="1270"></td>
        <td id="LC1270" class="blob-code blob-code-inner js-file-line">		g_LinkVector[li]-&gt;<span class="pl-c1">ComputeVSP</span>();</td>
      </tr>
      <tr>
        <td id="L1271" class="blob-num js-line-number" data-line-number="1271"></td>
        <td id="LC1271" class="blob-code blob-code-inner js-file-line">		<span class="pl-c">//g_LinkVector[li]-&gt;ThreeDetectorVehicleTrajectorySimulation();</span></td>
      </tr>
      <tr>
        <td id="L1272" class="blob-num js-line-number" data-line-number="1272"></td>
        <td id="LC1272" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L1273" class="blob-num js-line-number" data-line-number="1273"></td>
        <td id="LC1273" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1274" class="blob-num js-line-number" data-line-number="1274"></td>
        <td id="LC1274" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> Energy = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L1275" class="blob-num js-line-number" data-line-number="1275"></td>
        <td id="LC1275" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> CO2 = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L1276" class="blob-num js-line-number" data-line-number="1276"></td>
        <td id="LC1276" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> NOX = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L1277" class="blob-num js-line-number" data-line-number="1277"></td>
        <td id="LC1277" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> CO = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L1278" class="blob-num js-line-number" data-line-number="1278"></td>
        <td id="LC1278" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> HC = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L1279" class="blob-num js-line-number" data-line-number="1279"></td>
        <td id="LC1279" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1280" class="blob-num js-line-number" data-line-number="1280"></td>
        <td id="LC1280" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">// aggregation for each vehicle </span></td>
      </tr>
      <tr>
        <td id="L1281" class="blob-num js-line-number" data-line-number="1281"></td>
        <td id="LC1281" class="blob-code blob-code-inner js-file-line">	CVehicleEmissionResult emissionResult;</td>
      </tr>
      <tr>
        <td id="L1282" class="blob-num js-line-number" data-line-number="1282"></td>
        <td id="LC1282" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1283" class="blob-num js-line-number" data-line-number="1283"></td>
        <td id="LC1283" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">for</span> (iterVM = g_VehicleMap.<span class="pl-c1">begin</span>(); iterVM != g_VehicleMap.<span class="pl-c1">end</span>(); iterVM++)</td>
      </tr>
      <tr>
        <td id="L1284" class="blob-num js-line-number" data-line-number="1284"></td>
        <td id="LC1284" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L1285" class="blob-num js-line-number" data-line-number="1285"></td>
        <td id="LC1285" class="blob-code blob-code-inner js-file-line">		DTAVehicle* pVehicle = iterVM-&gt;second;</td>
      </tr>
      <tr>
        <td id="L1286" class="blob-num js-line-number" data-line-number="1286"></td>
        <td id="LC1286" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">if</span> (pVehicle-&gt;m_NodeSize &gt;= <span class="pl-c1">2</span> &amp;&amp; pVehicle-&gt;m_OperatingModeCount.<span class="pl-c1">size</span>() &gt; <span class="pl-c1">0</span>)  <span class="pl-c">// with physical path in the network</span></td>
      </tr>
      <tr>
        <td id="L1287" class="blob-num js-line-number" data-line-number="1287"></td>
        <td id="LC1287" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L1288" class="blob-num js-line-number" data-line-number="1288"></td>
        <td id="LC1288" class="blob-code blob-code-inner js-file-line">			<span class="pl-c1">CVehicleEmissionResult::CalculateEmissions</span>(pVehicle-&gt;m_VehicleType, pVehicle-&gt;m_OperatingModeCount, pVehicle-&gt;m_Age, emissionResult);</td>
      </tr>
      <tr>
        <td id="L1289" class="blob-num js-line-number" data-line-number="1289"></td>
        <td id="LC1289" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1290" class="blob-num js-line-number" data-line-number="1290"></td>
        <td id="LC1290" class="blob-code blob-code-inner js-file-line">			pVehicle-&gt;m_Emissions = emissionResult.<span class="pl-smi">CO2</span>;</td>
      </tr>
      <tr>
        <td id="L1291" class="blob-num js-line-number" data-line-number="1291"></td>
        <td id="LC1291" class="blob-code blob-code-inner js-file-line">			pVehicle-&gt;Energy = emissionResult.<span class="pl-smi">Energy</span>;</td>
      </tr>
      <tr>
        <td id="L1292" class="blob-num js-line-number" data-line-number="1292"></td>
        <td id="LC1292" class="blob-code blob-code-inner js-file-line">			pVehicle-&gt;CO2 = emissionResult.<span class="pl-smi">CO2</span>;</td>
      </tr>
      <tr>
        <td id="L1293" class="blob-num js-line-number" data-line-number="1293"></td>
        <td id="LC1293" class="blob-code blob-code-inner js-file-line">			pVehicle-&gt;NOX = emissionResult.<span class="pl-smi">NOX</span>;</td>
      </tr>
      <tr>
        <td id="L1294" class="blob-num js-line-number" data-line-number="1294"></td>
        <td id="LC1294" class="blob-code blob-code-inner js-file-line">			pVehicle-&gt;CO = emissionResult.<span class="pl-smi">CO</span>;</td>
      </tr>
      <tr>
        <td id="L1295" class="blob-num js-line-number" data-line-number="1295"></td>
        <td id="LC1295" class="blob-code blob-code-inner js-file-line">			pVehicle-&gt;HC = emissionResult.<span class="pl-smi">HC</span>;</td>
      </tr>
      <tr>
        <td id="L1296" class="blob-num js-line-number" data-line-number="1296"></td>
        <td id="LC1296" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1297" class="blob-num js-line-number" data-line-number="1297"></td>
        <td id="LC1297" class="blob-code blob-code-inner js-file-line">			Energy += emissionResult.<span class="pl-smi">Energy</span>;</td>
      </tr>
      <tr>
        <td id="L1298" class="blob-num js-line-number" data-line-number="1298"></td>
        <td id="LC1298" class="blob-code blob-code-inner js-file-line">			CO2 += emissionResult.<span class="pl-smi">CO2</span>;</td>
      </tr>
      <tr>
        <td id="L1299" class="blob-num js-line-number" data-line-number="1299"></td>
        <td id="LC1299" class="blob-code blob-code-inner js-file-line">			NOX += emissionResult.<span class="pl-smi">NOX</span>;</td>
      </tr>
      <tr>
        <td id="L1300" class="blob-num js-line-number" data-line-number="1300"></td>
        <td id="LC1300" class="blob-code blob-code-inner js-file-line">			CO += emissionResult.<span class="pl-smi">CO</span>;</td>
      </tr>
      <tr>
        <td id="L1301" class="blob-num js-line-number" data-line-number="1301"></td>
        <td id="LC1301" class="blob-code blob-code-inner js-file-line">			HC += emissionResult.<span class="pl-smi">HC</span>;</td>
      </tr>
      <tr>
        <td id="L1302" class="blob-num js-line-number" data-line-number="1302"></td>
        <td id="LC1302" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L1303" class="blob-num js-line-number" data-line-number="1303"></td>
        <td id="LC1303" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L1304" class="blob-num js-line-number" data-line-number="1304"></td>
        <td id="LC1304" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1305" class="blob-num js-line-number" data-line-number="1305"></td>
        <td id="LC1305" class="blob-code blob-code-inner js-file-line">	g_SimulationResult.<span class="pl-smi">Energy</span> = Energy;</td>
      </tr>
      <tr>
        <td id="L1306" class="blob-num js-line-number" data-line-number="1306"></td>
        <td id="LC1306" class="blob-code blob-code-inner js-file-line">	g_SimulationResult.<span class="pl-smi">CO2</span> = CO2;</td>
      </tr>
      <tr>
        <td id="L1307" class="blob-num js-line-number" data-line-number="1307"></td>
        <td id="LC1307" class="blob-code blob-code-inner js-file-line">	g_SimulationResult.<span class="pl-smi">NOX</span> = NOX;</td>
      </tr>
      <tr>
        <td id="L1308" class="blob-num js-line-number" data-line-number="1308"></td>
        <td id="LC1308" class="blob-code blob-code-inner js-file-line">	g_SimulationResult.<span class="pl-smi">CO</span> = CO;</td>
      </tr>
      <tr>
        <td id="L1309" class="blob-num js-line-number" data-line-number="1309"></td>
        <td id="LC1309" class="blob-code blob-code-inner js-file-line">	g_SimulationResult.<span class="pl-smi">HC</span> = HC;</td>
      </tr>
      <tr>
        <td id="L1310" class="blob-num js-line-number" data-line-number="1310"></td>
        <td id="LC1310" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1311" class="blob-num js-line-number" data-line-number="1311"></td>
        <td id="LC1311" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1312" class="blob-num js-line-number" data-line-number="1312"></td>
        <td id="LC1312" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">// aggregation for each link </span></td>
      </tr>
      <tr>
        <td id="L1313" class="blob-num js-line-number" data-line-number="1313"></td>
        <td id="LC1313" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">for</span> (<span class="pl-k">unsigned</span> li = <span class="pl-c1">0</span>; li&lt; g_LinkVector.<span class="pl-c1">size</span>(); li++)</td>
      </tr>
      <tr>
        <td id="L1314" class="blob-num js-line-number" data-line-number="1314"></td>
        <td id="LC1314" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L1315" class="blob-num js-line-number" data-line-number="1315"></td>
        <td id="LC1315" class="blob-code blob-code-inner js-file-line">		DTALink* pLink = g_LinkVector[li];</td>
      </tr>
      <tr>
        <td id="L1316" class="blob-num js-line-number" data-line-number="1316"></td>
        <td id="LC1316" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1317" class="blob-num js-line-number" data-line-number="1317"></td>
        <td id="LC1317" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">int</span> TimeSize = <span class="pl-c1">min</span>(pLink-&gt;m_LinkMOEAry.<span class="pl-c1">size</span>(), pLink-&gt;m_VehicleDataVector[<span class="pl-c1">0</span>].<span class="pl-smi">m_LaneEmissionVector</span>.<span class="pl-c1">size</span>());</td>
      </tr>
      <tr>
        <td id="L1318" class="blob-num js-line-number" data-line-number="1318"></td>
        <td id="LC1318" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">for</span> (<span class="pl-k">int</span> t = <span class="pl-c1">0</span>; t&lt; TimeSize; t++)</td>
      </tr>
      <tr>
        <td id="L1319" class="blob-num js-line-number" data-line-number="1319"></td>
        <td id="LC1319" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L1320" class="blob-num js-line-number" data-line-number="1320"></td>
        <td id="LC1320" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">for</span> (<span class="pl-k">int</span> LaneNo = <span class="pl-c1">0</span>; LaneNo &lt; pLink-&gt;m_OutflowNumLanes; LaneNo++)</td>
      </tr>
      <tr>
        <td id="L1321" class="blob-num js-line-number" data-line-number="1321"></td>
        <td id="LC1321" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L1322" class="blob-num js-line-number" data-line-number="1322"></td>
        <td id="LC1322" class="blob-code blob-code-inner js-file-line">				pLink-&gt;m_LinkMOEAry[t].<span class="pl-smi">Energy</span> += pLink-&gt;m_VehicleDataVector[LaneNo].<span class="pl-smi">m_LaneEmissionVector</span>[t].<span class="pl-smi">Energy</span>;</td>
      </tr>
      <tr>
        <td id="L1323" class="blob-num js-line-number" data-line-number="1323"></td>
        <td id="LC1323" class="blob-code blob-code-inner js-file-line">				pLink-&gt;m_LinkMOEAry[t].<span class="pl-smi">CO2</span> += pLink-&gt;m_VehicleDataVector[LaneNo].<span class="pl-smi">m_LaneEmissionVector</span>[t].<span class="pl-smi">CO2</span>;</td>
      </tr>
      <tr>
        <td id="L1324" class="blob-num js-line-number" data-line-number="1324"></td>
        <td id="LC1324" class="blob-code blob-code-inner js-file-line">				pLink-&gt;m_LinkMOEAry[t].<span class="pl-smi">NOX</span> += pLink-&gt;m_VehicleDataVector[LaneNo].<span class="pl-smi">m_LaneEmissionVector</span>[t].<span class="pl-smi">NOX</span>;</td>
      </tr>
      <tr>
        <td id="L1325" class="blob-num js-line-number" data-line-number="1325"></td>
        <td id="LC1325" class="blob-code blob-code-inner js-file-line">				pLink-&gt;m_LinkMOEAry[t].<span class="pl-smi">CO</span> += pLink-&gt;m_VehicleDataVector[LaneNo].<span class="pl-smi">m_LaneEmissionVector</span>[t].<span class="pl-smi">CO</span>;</td>
      </tr>
      <tr>
        <td id="L1326" class="blob-num js-line-number" data-line-number="1326"></td>
        <td id="LC1326" class="blob-code blob-code-inner js-file-line">				pLink-&gt;m_LinkMOEAry[t].<span class="pl-smi">HC</span> += pLink-&gt;m_VehicleDataVector[LaneNo].<span class="pl-smi">m_LaneEmissionVector</span>[t].<span class="pl-smi">HC</span>;</td>
      </tr>
      <tr>
        <td id="L1327" class="blob-num js-line-number" data-line-number="1327"></td>
        <td id="LC1327" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1328" class="blob-num js-line-number" data-line-number="1328"></td>
        <td id="LC1328" class="blob-code blob-code-inner js-file-line">				pLink-&gt;TotalEnergy += pLink-&gt;m_VehicleDataVector[LaneNo].<span class="pl-smi">m_LaneEmissionVector</span>[t].<span class="pl-smi">Energy</span>;</td>
      </tr>
      <tr>
        <td id="L1329" class="blob-num js-line-number" data-line-number="1329"></td>
        <td id="LC1329" class="blob-code blob-code-inner js-file-line">				pLink-&gt;TotalCO2 += pLink-&gt;m_VehicleDataVector[LaneNo].<span class="pl-smi">m_LaneEmissionVector</span>[t].<span class="pl-smi">CO2</span>;</td>
      </tr>
      <tr>
        <td id="L1330" class="blob-num js-line-number" data-line-number="1330"></td>
        <td id="LC1330" class="blob-code blob-code-inner js-file-line">				pLink-&gt;TotalNOX += pLink-&gt;m_VehicleDataVector[LaneNo].<span class="pl-smi">m_LaneEmissionVector</span>[t].<span class="pl-smi">NOX</span>;</td>
      </tr>
      <tr>
        <td id="L1331" class="blob-num js-line-number" data-line-number="1331"></td>
        <td id="LC1331" class="blob-code blob-code-inner js-file-line">				pLink-&gt;TotalCO += pLink-&gt;m_VehicleDataVector[LaneNo].<span class="pl-smi">m_LaneEmissionVector</span>[t].<span class="pl-smi">CO</span>;</td>
      </tr>
      <tr>
        <td id="L1332" class="blob-num js-line-number" data-line-number="1332"></td>
        <td id="LC1332" class="blob-code blob-code-inner js-file-line">				pLink-&gt;TotalHC += pLink-&gt;m_VehicleDataVector[LaneNo].<span class="pl-smi">m_LaneEmissionVector</span>[t].<span class="pl-smi">HC</span>;</td>
      </tr>
      <tr>
        <td id="L1333" class="blob-num js-line-number" data-line-number="1333"></td>
        <td id="LC1333" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L1334" class="blob-num js-line-number" data-line-number="1334"></td>
        <td id="LC1334" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L1335" class="blob-num js-line-number" data-line-number="1335"></td>
        <td id="LC1335" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L1336" class="blob-num js-line-number" data-line-number="1336"></td>
        <td id="LC1336" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">endif</span> </td>
      </tr>
      <tr>
        <td id="L1337" class="blob-num js-line-number" data-line-number="1337"></td>
        <td id="LC1337" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L1338" class="blob-num js-line-number" data-line-number="1338"></td>
        <td id="LC1338" class="blob-code blob-code-inner js-file-line"><span class="pl-k">void</span> <span class="pl-en">OutputEmissionData</span>()</td>
      </tr>
      <tr>
        <td id="L1339" class="blob-num js-line-number" data-line-number="1339"></td>
        <td id="LC1339" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L1340" class="blob-num js-line-number" data-line-number="1340"></td>
        <td id="LC1340" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">ifdef</span> _large_memory_usage</td>
      </tr>
      <tr>
        <td id="L1341" class="blob-num js-line-number" data-line-number="1341"></td>
        <td id="LC1341" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span>(g_TrafficFlowModelFlag ==  tfm_BPR)</td>
      </tr>
      <tr>
        <td id="L1342" class="blob-num js-line-number" data-line-number="1342"></td>
        <td id="LC1342" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">return</span>; <span class="pl-c">// no emission output for BPR function</span></td>
      </tr>
      <tr>
        <td id="L1343" class="blob-num js-line-number" data-line-number="1343"></td>
        <td id="LC1343" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1344" class="blob-num js-line-number" data-line-number="1344"></td>
        <td id="LC1344" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//ReadDrivingCycleSamplesByVehicleType(1,&quot;input_Cycle_PC_PT_LCT.csv&quot;,drivingCycleMap[1]);</span></td>
      </tr>
      <tr>
        <td id="L1345" class="blob-num js-line-number" data-line-number="1345"></td>
        <td id="LC1345" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//ReadDrivingCycleSamplesByVehicleType(4,&quot;input_Cycle_SST.csv&quot;,drivingCycleMap[4]);</span></td>
      </tr>
      <tr>
        <td id="L1346" class="blob-num js-line-number" data-line-number="1346"></td>
        <td id="LC1346" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//ReadDrivingCycleSamplesByVehicleType(5,&quot;input_CYCLE_LHT.csv&quot;,drivingCycleMap[5]);</span></td>
      </tr>
      <tr>
        <td id="L1347" class="blob-num js-line-number" data-line-number="1347"></td>
        <td id="LC1347" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1348" class="blob-num js-line-number" data-line-number="1348"></td>
        <td id="LC1348" class="blob-code blob-code-inner js-file-line">	<span class="pl-c1">g_CalculateEmissionMOE</span>();</td>
      </tr>
      <tr>
        <td id="L1349" class="blob-num js-line-number" data-line-number="1349"></td>
        <td id="LC1349" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1350" class="blob-num js-line-number" data-line-number="1350"></td>
        <td id="LC1350" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">// output speed data second by second</span></td>
      </tr>
      <tr>
        <td id="L1351" class="blob-num js-line-number" data-line-number="1351"></td>
        <td id="LC1351" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span>(g_OutputEmissionOperatingModeData)</td>
      </tr>
      <tr>
        <td id="L1352" class="blob-num js-line-number" data-line-number="1352"></td>
        <td id="LC1352" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L1353" class="blob-num js-line-number" data-line-number="1353"></td>
        <td id="LC1353" class="blob-code blob-code-inner js-file-line">		<span class="pl-c1">FILE</span>* st2;</td>
      </tr>
      <tr>
        <td id="L1354" class="blob-num js-line-number" data-line-number="1354"></td>
        <td id="LC1354" class="blob-code blob-code-inner js-file-line">		<span class="pl-c1">fopen_s</span>(&amp;st2,<span class="pl-s"><span class="pl-pds">&quot;</span>output_vehicle_operating_mode.csv<span class="pl-pds">&quot;</span></span>,<span class="pl-s"><span class="pl-pds">&quot;</span>w<span class="pl-pds">&quot;</span></span>);</td>
      </tr>
      <tr>
        <td id="L1355" class="blob-num js-line-number" data-line-number="1355"></td>
        <td id="LC1355" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1356" class="blob-num js-line-number" data-line-number="1356"></td>
        <td id="LC1356" class="blob-code blob-code-inner js-file-line">		<span class="pl-c1">FILE</span>* fpSampledEmissionsFile = <span class="pl-c1">NULL</span>;</td>
      </tr>
      <tr>
        <td id="L1357" class="blob-num js-line-number" data-line-number="1357"></td>
        <td id="LC1357" class="blob-code blob-code-inner js-file-line">		<span class="pl-c1">FILE</span>* fpSampledCycleEmissionRateFile = <span class="pl-c1">NULL</span>;</td>
      </tr>
      <tr>
        <td id="L1358" class="blob-num js-line-number" data-line-number="1358"></td>
        <td id="LC1358" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1359" class="blob-num js-line-number" data-line-number="1359"></td>
        <td id="LC1359" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">int</span> numOfSamples = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L1360" class="blob-num js-line-number" data-line-number="1360"></td>
        <td id="LC1360" class="blob-code blob-code-inner js-file-line">		<span class="pl-c">//int numOfVehicles = 0;</span></td>
      </tr>
      <tr>
        <td id="L1361" class="blob-num js-line-number" data-line-number="1361"></td>
        <td id="LC1361" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1362" class="blob-num js-line-number" data-line-number="1362"></td>
        <td id="LC1362" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">if</span> (g_OutputSecondBySecondEmissionData)</td>
      </tr>
      <tr>
        <td id="L1363" class="blob-num js-line-number" data-line-number="1363"></td>
        <td id="LC1363" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L1364" class="blob-num js-line-number" data-line-number="1364"></td>
        <td id="LC1364" class="blob-code blob-code-inner js-file-line">			<span class="pl-c1">fopen_s</span>(&amp;fpSampledEmissionsFile, <span class="pl-s"><span class="pl-pds">&quot;</span>output_sampled_vehicle_operating_mode.csv<span class="pl-pds">&quot;</span></span>,<span class="pl-s"><span class="pl-pds">&quot;</span>w<span class="pl-pds">&quot;</span></span>);</td>
      </tr>
      <tr>
        <td id="L1365" class="blob-num js-line-number" data-line-number="1365"></td>
        <td id="LC1365" class="blob-code blob-code-inner js-file-line">			<span class="pl-c1">fopen_s</span>(&amp;fpSampledCycleEmissionRateFile, <span class="pl-s"><span class="pl-pds">&quot;</span>output_sampled_vehicle_cycle_emission_rate.csv<span class="pl-pds">&quot;</span></span>,<span class="pl-s"><span class="pl-pds">&quot;</span>w<span class="pl-pds">&quot;</span></span>);</td>
      </tr>
      <tr>
        <td id="L1366" class="blob-num js-line-number" data-line-number="1366"></td>
        <td id="LC1366" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (fpSampledEmissionsFile)</td>
      </tr>
      <tr>
        <td id="L1367" class="blob-num js-line-number" data-line-number="1367"></td>
        <td id="LC1367" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L1368" class="blob-num js-line-number" data-line-number="1368"></td>
        <td id="LC1368" class="blob-code blob-code-inner js-file-line">				<span class="pl-c1">fprintf</span>(fpSampledEmissionsFile, <span class="pl-s"><span class="pl-pds">&quot;</span>Vehicle_id,Vehicle_Type,Vehicle_Age,Hour,Min,FromNodeNumber,ToNodeNumber,LinkType,TimeOnThisLinkInSecond,TripDurationInSecond,Speed(mph),Acceleration(mph/s),VSP,VSP_bin,Speed_bin,OperatingMode,Energy,CO2,NOX,CO,HC,Average_Speed(MPH),CO2_Rate(g/mi),NOX_Rate(g/mi),CO_Rate((g/mi),HC_Rate(g/mi)<span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>);</td>
      </tr>
      <tr>
        <td id="L1369" class="blob-num js-line-number" data-line-number="1369"></td>
        <td id="LC1369" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L1370" class="blob-num js-line-number" data-line-number="1370"></td>
        <td id="LC1370" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1371" class="blob-num js-line-number" data-line-number="1371"></td>
        <td id="LC1371" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (fpSampledCycleEmissionRateFile)</td>
      </tr>
      <tr>
        <td id="L1372" class="blob-num js-line-number" data-line-number="1372"></td>
        <td id="LC1372" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L1373" class="blob-num js-line-number" data-line-number="1373"></td>
        <td id="LC1373" class="blob-code blob-code-inner js-file-line">				<span class="pl-c1">fprintf</span>(fpSampledCycleEmissionRateFile, <span class="pl-s"><span class="pl-pds">&quot;</span>Vehicle_id,Vehicle_Type,Vehicle_Age,Average_Speed(MPH),CO2_Rate(g/mi),NOX_Rate(g/mi),CO_Rate((g/mi),HC_Rate(g/mi)<span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>);</td>
      </tr>
      <tr>
        <td id="L1374" class="blob-num js-line-number" data-line-number="1374"></td>
        <td id="LC1374" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L1375" class="blob-num js-line-number" data-line-number="1375"></td>
        <td id="LC1375" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L1376" class="blob-num js-line-number" data-line-number="1376"></td>
        <td id="LC1376" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1377" class="blob-num js-line-number" data-line-number="1377"></td>
        <td id="LC1377" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">if</span>(st2 != <span class="pl-c1">NULL</span>)</td>
      </tr>
      <tr>
        <td id="L1378" class="blob-num js-line-number" data-line-number="1378"></td>
        <td id="LC1378" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L1379" class="blob-num js-line-number" data-line-number="1379"></td>
        <td id="LC1379" class="blob-code blob-code-inner js-file-line">			cout &lt;&lt; <span class="pl-s"><span class="pl-pds">&quot;</span>writing output_sampled_vehicle_operating_mode.csv...<span class="pl-pds">&quot;</span></span> &lt;&lt; endl;</td>
      </tr>
      <tr>
        <td id="L1380" class="blob-num js-line-number" data-line-number="1380"></td>
        <td id="LC1380" class="blob-code blob-code-inner js-file-line">			std::map&lt;<span class="pl-k">int</span>, DTAVehicle*&gt;::iterator iterVM;</td>
      </tr>
      <tr>
        <td id="L1381" class="blob-num js-line-number" data-line-number="1381"></td>
        <td id="LC1381" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">//calculate the toll cost and emission cost</span></td>
      </tr>
      <tr>
        <td id="L1382" class="blob-num js-line-number" data-line-number="1382"></td>
        <td id="LC1382" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">for</span> (iterVM = g_VehicleMap.<span class="pl-c1">begin</span>(); iterVM != g_VehicleMap.<span class="pl-c1">end</span>(); iterVM++)</td>
      </tr>
      <tr>
        <td id="L1383" class="blob-num js-line-number" data-line-number="1383"></td>
        <td id="LC1383" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L1384" class="blob-num js-line-number" data-line-number="1384"></td>
        <td id="LC1384" class="blob-code blob-code-inner js-file-line">				DTAVehicle* pVehicle = iterVM-&gt;second;</td>
      </tr>
      <tr>
        <td id="L1385" class="blob-num js-line-number" data-line-number="1385"></td>
        <td id="LC1385" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1386" class="blob-num js-line-number" data-line-number="1386"></td>
        <td id="LC1386" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">/// print out second by second profile</span></td>
      </tr>
      <tr>
        <td id="L1387" class="blob-num js-line-number" data-line-number="1387"></td>
        <td id="LC1387" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span>(g_OutputSecondBySecondEmissionData)</td>
      </tr>
      <tr>
        <td id="L1388" class="blob-num js-line-number" data-line-number="1388"></td>
        <td id="LC1388" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L1389" class="blob-num js-line-number" data-line-number="1389"></td>
        <td id="LC1389" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">if</span> (pVehicle-&gt;m_bDetailedEmissionOutput == <span class="pl-c1">false</span>)</td>
      </tr>
      <tr>
        <td id="L1390" class="blob-num js-line-number" data-line-number="1390"></td>
        <td id="LC1390" class="blob-code blob-code-inner js-file-line">					{</td>
      </tr>
      <tr>
        <td id="L1391" class="blob-num js-line-number" data-line-number="1391"></td>
        <td id="LC1391" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">continue</span>;</td>
      </tr>
      <tr>
        <td id="L1392" class="blob-num js-line-number" data-line-number="1392"></td>
        <td id="LC1392" class="blob-code blob-code-inner js-file-line">					}</td>
      </tr>
      <tr>
        <td id="L1393" class="blob-num js-line-number" data-line-number="1393"></td>
        <td id="LC1393" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1394" class="blob-num js-line-number" data-line-number="1394"></td>
        <td id="LC1394" class="blob-code blob-code-inner js-file-line">					numOfSamples++;</td>
      </tr>
      <tr>
        <td id="L1395" class="blob-num js-line-number" data-line-number="1395"></td>
        <td id="LC1395" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1396" class="blob-num js-line-number" data-line-number="1396"></td>
        <td id="LC1396" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">if</span>(numOfSamples % <span class="pl-c1">500</span> == <span class="pl-c1">0</span>)</td>
      </tr>
      <tr>
        <td id="L1397" class="blob-num js-line-number" data-line-number="1397"></td>
        <td id="LC1397" class="blob-code blob-code-inner js-file-line">					{</td>
      </tr>
      <tr>
        <td id="L1398" class="blob-num js-line-number" data-line-number="1398"></td>
        <td id="LC1398" class="blob-code blob-code-inner js-file-line">						cout &lt;&lt; <span class="pl-s"><span class="pl-pds">&quot;</span> <span class="pl-pds">&quot;</span></span> &lt;&lt; numOfSamples &lt;&lt; <span class="pl-s"><span class="pl-pds">&quot;</span> emission samples have been generated ...<span class="pl-pds">&quot;</span></span> &lt;&lt; endl;</td>
      </tr>
      <tr>
        <td id="L1399" class="blob-num js-line-number" data-line-number="1399"></td>
        <td id="LC1399" class="blob-code blob-code-inner js-file-line">					}</td>
      </tr>
      <tr>
        <td id="L1400" class="blob-num js-line-number" data-line-number="1400"></td>
        <td id="LC1400" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1401" class="blob-num js-line-number" data-line-number="1401"></td>
        <td id="LC1401" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">int</span> VehicleSimulationStartTime;</td>
      </tr>
      <tr>
        <td id="L1402" class="blob-num js-line-number" data-line-number="1402"></td>
        <td id="LC1402" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">int</span> prevFromNodeNumber = -<span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L1403" class="blob-num js-line-number" data-line-number="1403"></td>
        <td id="LC1403" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">int</span> prevToNodeNumber = -<span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L1404" class="blob-num js-line-number" data-line-number="1404"></td>
        <td id="LC1404" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">int</span> prevLinkType = -<span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L1405" class="blob-num js-line-number" data-line-number="1405"></td>
        <td id="LC1405" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">int</span> currLinkType = -<span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L1406" class="blob-num js-line-number" data-line-number="1406"></td>
        <td id="LC1406" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">bool</span> isNewLink = <span class="pl-c1">false</span>;</td>
      </tr>
      <tr>
        <td id="L1407" class="blob-num js-line-number" data-line-number="1407"></td>
        <td id="LC1407" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1408" class="blob-num js-line-number" data-line-number="1408"></td>
        <td id="LC1408" class="blob-code blob-code-inner js-file-line">					<span class="pl-c">//ReconstructTrajectory(pVehicle-&gt;m_VehicleType, pVehicle-&gt;m_SpeedProfile);</span></td>
      </tr>
      <tr>
        <td id="L1409" class="blob-num js-line-number" data-line-number="1409"></td>
        <td id="LC1409" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">int</span> speedProfileSize = pVehicle-&gt;m_SpeedProfile.<span class="pl-c1">size</span>();</td>
      </tr>
      <tr>
        <td id="L1410" class="blob-num js-line-number" data-line-number="1410"></td>
        <td id="LC1410" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">int</span> speedProfileCounter = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L1411" class="blob-num js-line-number" data-line-number="1411"></td>
        <td id="LC1411" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">float</span> prev_speed = <span class="pl-c1">0</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L1412" class="blob-num js-line-number" data-line-number="1412"></td>
        <td id="LC1412" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1413" class="blob-num js-line-number" data-line-number="1413"></td>
        <td id="LC1413" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">float</span> prev_acceleration = -<span class="pl-c1">99999</span>.<span class="pl-c1">0f</span>; <span class="pl-c">//in meter/sec^2</span></td>
      </tr>
      <tr>
        <td id="L1414" class="blob-num js-line-number" data-line-number="1414"></td>
        <td id="LC1414" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">float</span> prev_prev_acceleration = -<span class="pl-c1">99999</span>.<span class="pl-c1">0f</span>; <span class="pl-c">//in meter/sec^2</span></td>
      </tr>
      <tr>
        <td id="L1415" class="blob-num js-line-number" data-line-number="1415"></td>
        <td id="LC1415" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1416" class="blob-num js-line-number" data-line-number="1416"></td>
        <td id="LC1416" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">float</span> totalCO2 = <span class="pl-c1">0</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L1417" class="blob-num js-line-number" data-line-number="1417"></td>
        <td id="LC1417" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">float</span> totalNOX = <span class="pl-c1">0</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L1418" class="blob-num js-line-number" data-line-number="1418"></td>
        <td id="LC1418" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">float</span> totalCO = <span class="pl-c1">0</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L1419" class="blob-num js-line-number" data-line-number="1419"></td>
        <td id="LC1419" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">float</span> totalHC = <span class="pl-c1">0</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L1420" class="blob-num js-line-number" data-line-number="1420"></td>
        <td id="LC1420" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1421" class="blob-num js-line-number" data-line-number="1421"></td>
        <td id="LC1421" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">float</span> AverageSpeed = <span class="pl-c1">0</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L1422" class="blob-num js-line-number" data-line-number="1422"></td>
        <td id="LC1422" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">if</span> (g_EmissionSmoothVehicleTrajectory)</td>
      </tr>
      <tr>
        <td id="L1423" class="blob-num js-line-number" data-line-number="1423"></td>
        <td id="LC1423" class="blob-code blob-code-inner js-file-line">					{</td>
      </tr>
      <tr>
        <td id="L1424" class="blob-num js-line-number" data-line-number="1424"></td>
        <td id="LC1424" class="blob-code blob-code-inner js-file-line">						<span class="pl-c1">SmoothVehicleTrajectory</span>(pVehicle-&gt;m_AgentID, pVehicle-&gt;m_SpeedProfile);</td>
      </tr>
      <tr>
        <td id="L1425" class="blob-num js-line-number" data-line-number="1425"></td>
        <td id="LC1425" class="blob-code blob-code-inner js-file-line">					}</td>
      </tr>
      <tr>
        <td id="L1426" class="blob-num js-line-number" data-line-number="1426"></td>
        <td id="LC1426" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1427" class="blob-num js-line-number" data-line-number="1427"></td>
        <td id="LC1427" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">for</span>(std::map&lt;<span class="pl-k">int</span>, VehicleSpeedProfileData&gt;::iterator iter_s  =  pVehicle-&gt;m_SpeedProfile.<span class="pl-c1">begin</span>();</td>
      </tr>
      <tr>
        <td id="L1428" class="blob-num js-line-number" data-line-number="1428"></td>
        <td id="LC1428" class="blob-code blob-code-inner js-file-line">						iter_s != pVehicle-&gt;m_SpeedProfile.<span class="pl-c1">end</span>();</td>
      </tr>
      <tr>
        <td id="L1429" class="blob-num js-line-number" data-line-number="1429"></td>
        <td id="LC1429" class="blob-code blob-code-inner js-file-line">						iter_s++, speedProfileCounter++)</td>
      </tr>
      <tr>
        <td id="L1430" class="blob-num js-line-number" data-line-number="1430"></td>
        <td id="LC1430" class="blob-code blob-code-inner js-file-line">					{  </td>
      </tr>
      <tr>
        <td id="L1431" class="blob-num js-line-number" data-line-number="1431"></td>
        <td id="LC1431" class="blob-code blob-code-inner js-file-line">						VehicleSpeedProfileData element = iter_s-&gt;second;</td>
      </tr>
      <tr>
        <td id="L1432" class="blob-num js-line-number" data-line-number="1432"></td>
        <td id="LC1432" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1433" class="blob-num js-line-number" data-line-number="1433"></td>
        <td id="LC1433" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">if</span> (fpSampledEmissionsFile)</td>
      </tr>
      <tr>
        <td id="L1434" class="blob-num js-line-number" data-line-number="1434"></td>
        <td id="LC1434" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L1435" class="blob-num js-line-number" data-line-number="1435"></td>
        <td id="LC1435" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//Get the link type by from and to node numbers</span></td>
      </tr>
      <tr>
        <td id="L1436" class="blob-num js-line-number" data-line-number="1436"></td>
        <td id="LC1436" class="blob-code blob-code-inner js-file-line">							<span class="pl-k">if</span> (prevFromNodeNumber != element.<span class="pl-smi">FromNodeNumber</span> || prevToNodeNumber != element.<span class="pl-smi">ToNodeNumber</span>)</td>
      </tr>
      <tr>
        <td id="L1437" class="blob-num js-line-number" data-line-number="1437"></td>
        <td id="LC1437" class="blob-code blob-code-inner js-file-line">							{</td>
      </tr>
      <tr>
        <td id="L1438" class="blob-num js-line-number" data-line-number="1438"></td>
        <td id="LC1438" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">if</span>(g_LinkMap.<span class="pl-c1">find</span>(<span class="pl-c1">GetLinkStringID</span>(element.<span class="pl-smi">FromNodeNumber</span>,element.<span class="pl-smi">ToNodeNumber</span>)) != g_LinkMap.<span class="pl-c1">end</span>())</td>
      </tr>
      <tr>
        <td id="L1439" class="blob-num js-line-number" data-line-number="1439"></td>
        <td id="LC1439" class="blob-code blob-code-inner js-file-line">								{</td>
      </tr>
      <tr>
        <td id="L1440" class="blob-num js-line-number" data-line-number="1440"></td>
        <td id="LC1440" class="blob-code blob-code-inner js-file-line">									DTALink* pLink = g_LinkMap[<span class="pl-c1">GetLinkStringID</span>(element.<span class="pl-smi">FromNodeNumber</span>,element.<span class="pl-smi">ToNodeNumber</span>)];</td>
      </tr>
      <tr>
        <td id="L1441" class="blob-num js-line-number" data-line-number="1441"></td>
        <td id="LC1441" class="blob-code blob-code-inner js-file-line">									currLinkType = prevLinkType = pLink-&gt;m_link_type;</td>
      </tr>
      <tr>
        <td id="L1442" class="blob-num js-line-number" data-line-number="1442"></td>
        <td id="LC1442" class="blob-code blob-code-inner js-file-line">								}</td>
      </tr>
      <tr>
        <td id="L1443" class="blob-num js-line-number" data-line-number="1443"></td>
        <td id="LC1443" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L1444" class="blob-num js-line-number" data-line-number="1444"></td>
        <td id="LC1444" class="blob-code blob-code-inner js-file-line">								{</td>
      </tr>
      <tr>
        <td id="L1445" class="blob-num js-line-number" data-line-number="1445"></td>
        <td id="LC1445" class="blob-code blob-code-inner js-file-line">									currLinkType = prevLinkType = -<span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L1446" class="blob-num js-line-number" data-line-number="1446"></td>
        <td id="LC1446" class="blob-code blob-code-inner js-file-line">								}</td>
      </tr>
      <tr>
        <td id="L1447" class="blob-num js-line-number" data-line-number="1447"></td>
        <td id="LC1447" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">//isNewLink = true;</span></td>
      </tr>
      <tr>
        <td id="L1448" class="blob-num js-line-number" data-line-number="1448"></td>
        <td id="LC1448" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1449" class="blob-num js-line-number" data-line-number="1449"></td>
        <td id="LC1449" class="blob-code blob-code-inner js-file-line">								prevFromNodeNumber = element.<span class="pl-smi">FromNodeNumber</span>;</td>
      </tr>
      <tr>
        <td id="L1450" class="blob-num js-line-number" data-line-number="1450"></td>
        <td id="LC1450" class="blob-code blob-code-inner js-file-line">								prevToNodeNumber = element.<span class="pl-smi">ToNodeNumber</span>;</td>
      </tr>
      <tr>
        <td id="L1451" class="blob-num js-line-number" data-line-number="1451"></td>
        <td id="LC1451" class="blob-code blob-code-inner js-file-line">							}</td>
      </tr>
      <tr>
        <td id="L1452" class="blob-num js-line-number" data-line-number="1452"></td>
        <td id="LC1452" class="blob-code blob-code-inner js-file-line">							<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L1453" class="blob-num js-line-number" data-line-number="1453"></td>
        <td id="LC1453" class="blob-code blob-code-inner js-file-line">							{</td>
      </tr>
      <tr>
        <td id="L1454" class="blob-num js-line-number" data-line-number="1454"></td>
        <td id="LC1454" class="blob-code blob-code-inner js-file-line">								isNewLink = <span class="pl-c1">false</span>;								</td>
      </tr>
      <tr>
        <td id="L1455" class="blob-num js-line-number" data-line-number="1455"></td>
        <td id="LC1455" class="blob-code blob-code-inner js-file-line">							}</td>
      </tr>
      <tr>
        <td id="L1456" class="blob-num js-line-number" data-line-number="1456"></td>
        <td id="LC1456" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1457" class="blob-num js-line-number" data-line-number="1457"></td>
        <td id="LC1457" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">// If it is the first second on the first link or the last second on the last link, recalculate the acceleration rate, VSP and emissions</span></td>
      </tr>
      <tr>
        <td id="L1458" class="blob-num js-line-number" data-line-number="1458"></td>
        <td id="LC1458" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//if (iter_s == pVehicle-&gt;m_SpeedProfile.begin() || speedProfileCounter + 1 == speedProfileSize) </span></td>
      </tr>
      <tr>
        <td id="L1459" class="blob-num js-line-number" data-line-number="1459"></td>
        <td id="LC1459" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//{</span></td>
      </tr>
      <tr>
        <td id="L1460" class="blob-num js-line-number" data-line-number="1460"></td>
        <td id="LC1460" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	isNewLink = false;</span></td>
      </tr>
      <tr>
        <td id="L1461" class="blob-num js-line-number" data-line-number="1461"></td>
        <td id="LC1461" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1462" class="blob-num js-line-number" data-line-number="1462"></td>
        <td id="LC1462" class="blob-code blob-code-inner js-file-line">							VehicleSimulationStartTime = pVehicle-&gt;m_SpeedProfile.<span class="pl-c1">begin</span>()-&gt;first;</td>
      </tr>
      <tr>
        <td id="L1463" class="blob-num js-line-number" data-line-number="1463"></td>
        <td id="LC1463" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1464" class="blob-num js-line-number" data-line-number="1464"></td>
        <td id="LC1464" class="blob-code blob-code-inner js-file-line">							<span class="pl-k">float</span> acceleration = iter_s-&gt;second.<span class="pl-smi">Acceleration</span> * <span class="pl-c1">0</span>.<span class="pl-c1">44704f</span>;</td>
      </tr>
      <tr>
        <td id="L1465" class="blob-num js-line-number" data-line-number="1465"></td>
        <td id="LC1465" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">//if (iter_s == pVehicle-&gt;m_SpeedProfile.begin())</span></td>
      </tr>
      <tr>
        <td id="L1466" class="blob-num js-line-number" data-line-number="1466"></td>
        <td id="LC1466" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">//{</span></td>
      </tr>
      <tr>
        <td id="L1467" class="blob-num js-line-number" data-line-number="1467"></td>
        <td id="LC1467" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">//	acceleration = iter_s-&gt;second.Speed * 0.44704f; //mph to meter/sec^2</span></td>
      </tr>
      <tr>
        <td id="L1468" class="blob-num js-line-number" data-line-number="1468"></td>
        <td id="LC1468" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">//}</span></td>
      </tr>
      <tr>
        <td id="L1469" class="blob-num js-line-number" data-line-number="1469"></td>
        <td id="LC1469" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1470" class="blob-num js-line-number" data-line-number="1470"></td>
        <td id="LC1470" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">//if (speedProfileCounter + 1 == speedProfileSize)</span></td>
      </tr>
      <tr>
        <td id="L1471" class="blob-num js-line-number" data-line-number="1471"></td>
        <td id="LC1471" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">//{</span></td>
      </tr>
      <tr>
        <td id="L1472" class="blob-num js-line-number" data-line-number="1472"></td>
        <td id="LC1472" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">//	acceleration = -iter_s-&gt;second.Speed * 0.44704f;</span></td>
      </tr>
      <tr>
        <td id="L1473" class="blob-num js-line-number" data-line-number="1473"></td>
        <td id="LC1473" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">//	element.Speed = iter_s-&gt;second.Speed = 0.0f;</span></td>
      </tr>
      <tr>
        <td id="L1474" class="blob-num js-line-number" data-line-number="1474"></td>
        <td id="LC1474" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">//}</span></td>
      </tr>
      <tr>
        <td id="L1475" class="blob-num js-line-number" data-line-number="1475"></td>
        <td id="LC1475" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1476" class="blob-num js-line-number" data-line-number="1476"></td>
        <td id="LC1476" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">float</span> vsp = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L1477" class="blob-num js-line-number" data-line-number="1477"></td>
        <td id="LC1477" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">int</span> OperatingMode = <span class="pl-c1">ComputeOperatingModeFromSpeed</span>(vsp,iter_s-&gt;second.<span class="pl-smi">Speed</span> * <span class="pl-c1">0</span>.<span class="pl-c1">44704f</span>, acceleration,<span class="pl-c1">0</span>,pVehicle-&gt;m_VehicleType);</td>
      </tr>
      <tr>
        <td id="L1478" class="blob-num js-line-number" data-line-number="1478"></td>
        <td id="LC1478" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1479" class="blob-num js-line-number" data-line-number="1479"></td>
        <td id="LC1479" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">if</span> (acceleration &lt;= -<span class="pl-c1">2</span>)</td>
      </tr>
      <tr>
        <td id="L1480" class="blob-num js-line-number" data-line-number="1480"></td>
        <td id="LC1480" class="blob-code blob-code-inner js-file-line">								{</td>
      </tr>
      <tr>
        <td id="L1481" class="blob-num js-line-number" data-line-number="1481"></td>
        <td id="LC1481" class="blob-code blob-code-inner js-file-line">									OperatingMode = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L1482" class="blob-num js-line-number" data-line-number="1482"></td>
        <td id="LC1482" class="blob-code blob-code-inner js-file-line">								}</td>
      </tr>
      <tr>
        <td id="L1483" class="blob-num js-line-number" data-line-number="1483"></td>
        <td id="LC1483" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L1484" class="blob-num js-line-number" data-line-number="1484"></td>
        <td id="LC1484" class="blob-code blob-code-inner js-file-line">								{</td>
      </tr>
      <tr>
        <td id="L1485" class="blob-num js-line-number" data-line-number="1485"></td>
        <td id="LC1485" class="blob-code blob-code-inner js-file-line">									<span class="pl-k">if</span> (prev_prev_acceleration &lt; -<span class="pl-c1">1</span> &amp;&amp; prev_acceleration &lt; -<span class="pl-c1">1</span> &amp;&amp; acceleration &lt; -<span class="pl-c1">1</span>)</td>
      </tr>
      <tr>
        <td id="L1486" class="blob-num js-line-number" data-line-number="1486"></td>
        <td id="LC1486" class="blob-code blob-code-inner js-file-line">									{</td>
      </tr>
      <tr>
        <td id="L1487" class="blob-num js-line-number" data-line-number="1487"></td>
        <td id="LC1487" class="blob-code blob-code-inner js-file-line">										<span class="pl-k">if</span> (prev_prev_acceleration != -<span class="pl-c1">99999</span>.<span class="pl-c1">0f</span> || prev_acceleration != -<span class="pl-c1">99999</span>.<span class="pl-c1">0f</span>)</td>
      </tr>
      <tr>
        <td id="L1488" class="blob-num js-line-number" data-line-number="1488"></td>
        <td id="LC1488" class="blob-code blob-code-inner js-file-line">										{</td>
      </tr>
      <tr>
        <td id="L1489" class="blob-num js-line-number" data-line-number="1489"></td>
        <td id="LC1489" class="blob-code blob-code-inner js-file-line">											OperatingMode = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L1490" class="blob-num js-line-number" data-line-number="1490"></td>
        <td id="LC1490" class="blob-code blob-code-inner js-file-line">										}</td>
      </tr>
      <tr>
        <td id="L1491" class="blob-num js-line-number" data-line-number="1491"></td>
        <td id="LC1491" class="blob-code blob-code-inner js-file-line">									}</td>
      </tr>
      <tr>
        <td id="L1492" class="blob-num js-line-number" data-line-number="1492"></td>
        <td id="LC1492" class="blob-code blob-code-inner js-file-line">								}</td>
      </tr>
      <tr>
        <td id="L1493" class="blob-num js-line-number" data-line-number="1493"></td>
        <td id="LC1493" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1494" class="blob-num js-line-number" data-line-number="1494"></td>
        <td id="LC1494" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">if</span> (iter_s-&gt;second.<span class="pl-smi">Speed</span> &gt;= -<span class="pl-c1">1</span> &amp;&amp; iter_s-&gt;second.<span class="pl-smi">Speed</span> &lt; <span class="pl-c1">1</span>)</td>
      </tr>
      <tr>
        <td id="L1495" class="blob-num js-line-number" data-line-number="1495"></td>
        <td id="LC1495" class="blob-code blob-code-inner js-file-line">								{</td>
      </tr>
      <tr>
        <td id="L1496" class="blob-num js-line-number" data-line-number="1496"></td>
        <td id="LC1496" class="blob-code blob-code-inner js-file-line">									OperatingMode = <span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L1497" class="blob-num js-line-number" data-line-number="1497"></td>
        <td id="LC1497" class="blob-code blob-code-inner js-file-line">								}</td>
      </tr>
      <tr>
        <td id="L1498" class="blob-num js-line-number" data-line-number="1498"></td>
        <td id="LC1498" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1499" class="blob-num js-line-number" data-line-number="1499"></td>
        <td id="LC1499" class="blob-code blob-code-inner js-file-line">								pVehicle-&gt;m_OperatingModeCount[element.<span class="pl-smi">OperationMode</span>]--;</td>
      </tr>
      <tr>
        <td id="L1500" class="blob-num js-line-number" data-line-number="1500"></td>
        <td id="LC1500" class="blob-code blob-code-inner js-file-line">								pVehicle-&gt;m_OperatingModeCount[OperatingMode]++;							</td>
      </tr>
      <tr>
        <td id="L1501" class="blob-num js-line-number" data-line-number="1501"></td>
        <td id="LC1501" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1502" class="blob-num js-line-number" data-line-number="1502"></td>
        <td id="LC1502" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">int</span> vehicle_type = pVehicle-&gt;m_VehicleType;</td>
      </tr>
      <tr>
        <td id="L1503" class="blob-num js-line-number" data-line-number="1503"></td>
        <td id="LC1503" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">int</span> age = pVehicle-&gt;m_Age;</td>
      </tr>
      <tr>
        <td id="L1504" class="blob-num js-line-number" data-line-number="1504"></td>
        <td id="LC1504" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">float</span> Energy = EmissionRateData[vehicle_type][OperatingMode][age].<span class="pl-smi">meanBaseRate_TotalEnergy</span>/<span class="pl-c1">3600</span>;</td>
      </tr>
      <tr>
        <td id="L1505" class="blob-num js-line-number" data-line-number="1505"></td>
        <td id="LC1505" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">float</span> CO2 = EmissionRateData[vehicle_type][OperatingMode][age].<span class="pl-smi">meanBaseRate_CO2</span>/<span class="pl-c1">3600</span>;</td>
      </tr>
      <tr>
        <td id="L1506" class="blob-num js-line-number" data-line-number="1506"></td>
        <td id="LC1506" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">float</span> NOX = EmissionRateData[vehicle_type][OperatingMode][age].<span class="pl-smi">meanBaseRate_NOX</span>/<span class="pl-c1">3600</span>;</td>
      </tr>
      <tr>
        <td id="L1507" class="blob-num js-line-number" data-line-number="1507"></td>
        <td id="LC1507" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">float</span> CO = EmissionRateData[vehicle_type][OperatingMode][age].<span class="pl-smi">meanBaseRate_CO</span>/<span class="pl-c1">3600</span>;</td>
      </tr>
      <tr>
        <td id="L1508" class="blob-num js-line-number" data-line-number="1508"></td>
        <td id="LC1508" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">float</span> HC = EmissionRateData[vehicle_type][OperatingMode][age].<span class="pl-smi">meanBaseRate_HC</span>/<span class="pl-c1">3600</span>;</td>
      </tr>
      <tr>
        <td id="L1509" class="blob-num js-line-number" data-line-number="1509"></td>
        <td id="LC1509" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1510" class="blob-num js-line-number" data-line-number="1510"></td>
        <td id="LC1510" class="blob-code blob-code-inner js-file-line">								element.<span class="pl-smi">Acceleration</span> = iter_s-&gt;second.<span class="pl-smi">Acceleration</span>; <span class="pl-c">//meter/sec^2 to feet/sec^2</span></td>
      </tr>
      <tr>
        <td id="L1511" class="blob-num js-line-number" data-line-number="1511"></td>
        <td id="LC1511" class="blob-code blob-code-inner js-file-line">								element.<span class="pl-smi">VSP</span> = vsp;</td>
      </tr>
      <tr>
        <td id="L1512" class="blob-num js-line-number" data-line-number="1512"></td>
        <td id="LC1512" class="blob-code blob-code-inner js-file-line">								element.<span class="pl-smi">OperationMode</span> = OperatingMode;</td>
      </tr>
      <tr>
        <td id="L1513" class="blob-num js-line-number" data-line-number="1513"></td>
        <td id="LC1513" class="blob-code blob-code-inner js-file-line">								element.<span class="pl-smi">VSPBinNo</span> = <span class="pl-c1">GetVSPBinNo</span>(vsp, element.<span class="pl-smi">Speed</span>);</td>
      </tr>
      <tr>
        <td id="L1514" class="blob-num js-line-number" data-line-number="1514"></td>
        <td id="LC1514" class="blob-code blob-code-inner js-file-line">								element.<span class="pl-smi">SpeedBinNo</span> = <span class="pl-c1">GetSpeedBinNo</span>(element.<span class="pl-smi">Speed</span>);</td>
      </tr>
      <tr>
        <td id="L1515" class="blob-num js-line-number" data-line-number="1515"></td>
        <td id="LC1515" class="blob-code blob-code-inner js-file-line">								element.<span class="pl-smi">Energy</span> = Energy;</td>
      </tr>
      <tr>
        <td id="L1516" class="blob-num js-line-number" data-line-number="1516"></td>
        <td id="LC1516" class="blob-code blob-code-inner js-file-line">								element.<span class="pl-smi">CO2</span> = CO2;</td>
      </tr>
      <tr>
        <td id="L1517" class="blob-num js-line-number" data-line-number="1517"></td>
        <td id="LC1517" class="blob-code blob-code-inner js-file-line">								element.<span class="pl-smi">NOX</span> = NOX;</td>
      </tr>
      <tr>
        <td id="L1518" class="blob-num js-line-number" data-line-number="1518"></td>
        <td id="LC1518" class="blob-code blob-code-inner js-file-line">								element.<span class="pl-smi">CO</span> = CO;</td>
      </tr>
      <tr>
        <td id="L1519" class="blob-num js-line-number" data-line-number="1519"></td>
        <td id="LC1519" class="blob-code blob-code-inner js-file-line">								element.<span class="pl-smi">HC</span> = HC;</td>
      </tr>
      <tr>
        <td id="L1520" class="blob-num js-line-number" data-line-number="1520"></td>
        <td id="LC1520" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//}</span></td>
      </tr>
      <tr>
        <td id="L1521" class="blob-num js-line-number" data-line-number="1521"></td>
        <td id="LC1521" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1522" class="blob-num js-line-number" data-line-number="1522"></td>
        <td id="LC1522" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">// First second on the new Link, recalculate the acceleration rate, VSP and emissions</span></td>
      </tr>
      <tr>
        <td id="L1523" class="blob-num js-line-number" data-line-number="1523"></td>
        <td id="LC1523" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//if (isNewLink)</span></td>
      </tr>
      <tr>
        <td id="L1524" class="blob-num js-line-number" data-line-number="1524"></td>
        <td id="LC1524" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//{</span></td>
      </tr>
      <tr>
        <td id="L1525" class="blob-num js-line-number" data-line-number="1525"></td>
        <td id="LC1525" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	std::map&lt;int, VehicleSpeedProfileData&gt;::iterator prev_iter_s = --iter_s;</span></td>
      </tr>
      <tr>
        <td id="L1526" class="blob-num js-line-number" data-line-number="1526"></td>
        <td id="LC1526" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	iter_s++;</span></td>
      </tr>
      <tr>
        <td id="L1527" class="blob-num js-line-number" data-line-number="1527"></td>
        <td id="LC1527" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	float acceleration = (iter_s-&gt;second.Speed - prev_speed) * 0.44704f;</span></td>
      </tr>
      <tr>
        <td id="L1528" class="blob-num js-line-number" data-line-number="1528"></td>
        <td id="LC1528" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	float vsp = 0;</span></td>
      </tr>
      <tr>
        <td id="L1529" class="blob-num js-line-number" data-line-number="1529"></td>
        <td id="LC1529" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	int OperatingMode = ComputeOperatingModeFromSpeed(vsp,iter_s-&gt;second.Speed * 0.44704f, acceleration,0,pVehicle-&gt;m_VehicleType);</span></td>
      </tr>
      <tr>
        <td id="L1530" class="blob-num js-line-number" data-line-number="1530"></td>
        <td id="LC1530" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	if (acceleration &lt;= -2)</span></td>
      </tr>
      <tr>
        <td id="L1531" class="blob-num js-line-number" data-line-number="1531"></td>
        <td id="LC1531" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	{</span></td>
      </tr>
      <tr>
        <td id="L1532" class="blob-num js-line-number" data-line-number="1532"></td>
        <td id="LC1532" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//		OperatingMode = 0;</span></td>
      </tr>
      <tr>
        <td id="L1533" class="blob-num js-line-number" data-line-number="1533"></td>
        <td id="LC1533" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	}</span></td>
      </tr>
      <tr>
        <td id="L1534" class="blob-num js-line-number" data-line-number="1534"></td>
        <td id="LC1534" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	else</span></td>
      </tr>
      <tr>
        <td id="L1535" class="blob-num js-line-number" data-line-number="1535"></td>
        <td id="LC1535" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	{</span></td>
      </tr>
      <tr>
        <td id="L1536" class="blob-num js-line-number" data-line-number="1536"></td>
        <td id="LC1536" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//		if (prev_prev_acceleration &lt; -1 &amp;&amp; prev_acceleration &lt; -1 &amp;&amp; acceleration &lt; -1)</span></td>
      </tr>
      <tr>
        <td id="L1537" class="blob-num js-line-number" data-line-number="1537"></td>
        <td id="LC1537" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//		{</span></td>
      </tr>
      <tr>
        <td id="L1538" class="blob-num js-line-number" data-line-number="1538"></td>
        <td id="LC1538" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//			if (prev_prev_acceleration != -99999.0f || prev_acceleration != -99999.0f)</span></td>
      </tr>
      <tr>
        <td id="L1539" class="blob-num js-line-number" data-line-number="1539"></td>
        <td id="LC1539" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//			{</span></td>
      </tr>
      <tr>
        <td id="L1540" class="blob-num js-line-number" data-line-number="1540"></td>
        <td id="LC1540" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//				OperatingMode = 0;</span></td>
      </tr>
      <tr>
        <td id="L1541" class="blob-num js-line-number" data-line-number="1541"></td>
        <td id="LC1541" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//			}</span></td>
      </tr>
      <tr>
        <td id="L1542" class="blob-num js-line-number" data-line-number="1542"></td>
        <td id="LC1542" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//		}</span></td>
      </tr>
      <tr>
        <td id="L1543" class="blob-num js-line-number" data-line-number="1543"></td>
        <td id="LC1543" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	}</span></td>
      </tr>
      <tr>
        <td id="L1544" class="blob-num js-line-number" data-line-number="1544"></td>
        <td id="LC1544" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1545" class="blob-num js-line-number" data-line-number="1545"></td>
        <td id="LC1545" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	if (iter_s-&gt;second.Speed &gt;= -1 &amp;&amp; iter_s-&gt;second.Speed &lt; 1)</span></td>
      </tr>
      <tr>
        <td id="L1546" class="blob-num js-line-number" data-line-number="1546"></td>
        <td id="LC1546" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	{</span></td>
      </tr>
      <tr>
        <td id="L1547" class="blob-num js-line-number" data-line-number="1547"></td>
        <td id="LC1547" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//		OperatingMode = 1;</span></td>
      </tr>
      <tr>
        <td id="L1548" class="blob-num js-line-number" data-line-number="1548"></td>
        <td id="LC1548" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	}</span></td>
      </tr>
      <tr>
        <td id="L1549" class="blob-num js-line-number" data-line-number="1549"></td>
        <td id="LC1549" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1550" class="blob-num js-line-number" data-line-number="1550"></td>
        <td id="LC1550" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	pVehicle-&gt;m_OperatingModeCount[element.OperationMode]--;</span></td>
      </tr>
      <tr>
        <td id="L1551" class="blob-num js-line-number" data-line-number="1551"></td>
        <td id="LC1551" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	pVehicle-&gt;m_OperatingModeCount[OperatingMode]++;							</span></td>
      </tr>
      <tr>
        <td id="L1552" class="blob-num js-line-number" data-line-number="1552"></td>
        <td id="LC1552" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1553" class="blob-num js-line-number" data-line-number="1553"></td>
        <td id="LC1553" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	int vehicle_type = pVehicle-&gt;m_VehicleType;</span></td>
      </tr>
      <tr>
        <td id="L1554" class="blob-num js-line-number" data-line-number="1554"></td>
        <td id="LC1554" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	int age = pVehicle-&gt;m_Age;</span></td>
      </tr>
      <tr>
        <td id="L1555" class="blob-num js-line-number" data-line-number="1555"></td>
        <td id="LC1555" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	float Energy = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_TotalEnergy/3600;</span></td>
      </tr>
      <tr>
        <td id="L1556" class="blob-num js-line-number" data-line-number="1556"></td>
        <td id="LC1556" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	float CO2 = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_CO2/3600;</span></td>
      </tr>
      <tr>
        <td id="L1557" class="blob-num js-line-number" data-line-number="1557"></td>
        <td id="LC1557" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	float NOX = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_NOX/3600;</span></td>
      </tr>
      <tr>
        <td id="L1558" class="blob-num js-line-number" data-line-number="1558"></td>
        <td id="LC1558" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	float CO = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_CO/3600;</span></td>
      </tr>
      <tr>
        <td id="L1559" class="blob-num js-line-number" data-line-number="1559"></td>
        <td id="LC1559" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	float HC = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_HC/3600;</span></td>
      </tr>
      <tr>
        <td id="L1560" class="blob-num js-line-number" data-line-number="1560"></td>
        <td id="LC1560" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1561" class="blob-num js-line-number" data-line-number="1561"></td>
        <td id="LC1561" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	element.Acceleration = acceleration * 3.28084f; // meter/sec^2 to feet/sec^2</span></td>
      </tr>
      <tr>
        <td id="L1562" class="blob-num js-line-number" data-line-number="1562"></td>
        <td id="LC1562" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	element.VSP = vsp;</span></td>
      </tr>
      <tr>
        <td id="L1563" class="blob-num js-line-number" data-line-number="1563"></td>
        <td id="LC1563" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	element.OperationMode = OperatingMode;</span></td>
      </tr>
      <tr>
        <td id="L1564" class="blob-num js-line-number" data-line-number="1564"></td>
        <td id="LC1564" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	element.VSPBinNo = GetVSPBinNo(vsp, element.Speed);</span></td>
      </tr>
      <tr>
        <td id="L1565" class="blob-num js-line-number" data-line-number="1565"></td>
        <td id="LC1565" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	element.SpeedBinNo = GetSpeedBinNo(element.Speed);</span></td>
      </tr>
      <tr>
        <td id="L1566" class="blob-num js-line-number" data-line-number="1566"></td>
        <td id="LC1566" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	element.Energy = Energy;</span></td>
      </tr>
      <tr>
        <td id="L1567" class="blob-num js-line-number" data-line-number="1567"></td>
        <td id="LC1567" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	element.CO2 = CO2;</span></td>
      </tr>
      <tr>
        <td id="L1568" class="blob-num js-line-number" data-line-number="1568"></td>
        <td id="LC1568" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	element.NOX = NOX;</span></td>
      </tr>
      <tr>
        <td id="L1569" class="blob-num js-line-number" data-line-number="1569"></td>
        <td id="LC1569" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	element.CO = CO;</span></td>
      </tr>
      <tr>
        <td id="L1570" class="blob-num js-line-number" data-line-number="1570"></td>
        <td id="LC1570" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//	element.HC = HC;</span></td>
      </tr>
      <tr>
        <td id="L1571" class="blob-num js-line-number" data-line-number="1571"></td>
        <td id="LC1571" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//}</span></td>
      </tr>
      <tr>
        <td id="L1572" class="blob-num js-line-number" data-line-number="1572"></td>
        <td id="LC1572" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1573" class="blob-num js-line-number" data-line-number="1573"></td>
        <td id="LC1573" class="blob-code blob-code-inner js-file-line">							<span class="pl-c1">fprintf</span>(fpSampledEmissionsFile, <span class="pl-s"><span class="pl-pds">&quot;</span><span class="pl-c1">%d</span>,<span class="pl-c1">%d</span>,<span class="pl-c1">%d</span>,<span class="pl-c1">%d</span>,<span class="pl-c1">%d</span>,<span class="pl-c1">%d</span>,<span class="pl-c1">%d</span>,<span class="pl-c1">%s</span>,<span class="pl-c1">%d</span>,<span class="pl-c1">%d</span>,<span class="pl-c1">%5.2f</span><span class="pl-pds">&quot;</span></span>,</td>
      </tr>
      <tr>
        <td id="L1574" class="blob-num js-line-number" data-line-number="1574"></td>
        <td id="LC1574" class="blob-code blob-code-inner js-file-line">								pVehicle-&gt;m_AgentID,</td>
      </tr>
      <tr>
        <td id="L1575" class="blob-num js-line-number" data-line-number="1575"></td>
        <td id="LC1575" class="blob-code blob-code-inner js-file-line">								pVehicle-&gt;m_VehicleType,</td>
      </tr>
      <tr>
        <td id="L1576" class="blob-num js-line-number" data-line-number="1576"></td>
        <td id="LC1576" class="blob-code blob-code-inner js-file-line">								pVehicle-&gt;m_Age,</td>
      </tr>
      <tr>
        <td id="L1577" class="blob-num js-line-number" data-line-number="1577"></td>
        <td id="LC1577" class="blob-code blob-code-inner js-file-line">								(<span class="pl-k">int</span>)(iter_s-&gt;first / <span class="pl-c1">3600</span> / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND),</td>
      </tr>
      <tr>
        <td id="L1578" class="blob-num js-line-number" data-line-number="1578"></td>
        <td id="LC1578" class="blob-code blob-code-inner js-file-line">								((<span class="pl-k">int</span>)(iter_s-&gt;first / <span class="pl-c1">60</span> / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND)) % <span class="pl-c1">60</span>,								</td>
      </tr>
      <tr>
        <td id="L1579" class="blob-num js-line-number" data-line-number="1579"></td>
        <td id="LC1579" class="blob-code blob-code-inner js-file-line">								element.<span class="pl-smi">FromNodeNumber</span>,</td>
      </tr>
      <tr>
        <td id="L1580" class="blob-num js-line-number" data-line-number="1580"></td>
        <td id="LC1580" class="blob-code blob-code-inner js-file-line">								element.<span class="pl-smi">ToNodeNumber</span>,</td>
      </tr>
      <tr>
        <td id="L1581" class="blob-num js-line-number" data-line-number="1581"></td>
        <td id="LC1581" class="blob-code blob-code-inner js-file-line">								(currLinkType == -<span class="pl-c1">1</span>) ? <span class="pl-s"><span class="pl-pds">&quot;</span>Unknown<span class="pl-pds">&quot;</span></span>:g_LinkTypeMap[currLinkType].<span class="pl-smi">link_type_name</span>.<span class="pl-c1">c_str</span>(),</td>
      </tr>
      <tr>
        <td id="L1582" class="blob-num js-line-number" data-line-number="1582"></td>
        <td id="LC1582" class="blob-code blob-code-inner js-file-line">								element.<span class="pl-smi">TimeOnThisLinkInSecond</span>,</td>
      </tr>
      <tr>
        <td id="L1583" class="blob-num js-line-number" data-line-number="1583"></td>
        <td id="LC1583" class="blob-code blob-code-inner js-file-line">								(<span class="pl-k">int</span>)((iter_s-&gt;first - VehicleSimulationStartTime) / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND),</td>
      </tr>
      <tr>
        <td id="L1584" class="blob-num js-line-number" data-line-number="1584"></td>
        <td id="LC1584" class="blob-code blob-code-inner js-file-line">								element.<span class="pl-smi">Speed</span></td>
      </tr>
      <tr>
        <td id="L1585" class="blob-num js-line-number" data-line-number="1585"></td>
        <td id="LC1585" class="blob-code blob-code-inner js-file-line">								);</td>
      </tr>
      <tr>
        <td id="L1586" class="blob-num js-line-number" data-line-number="1586"></td>
        <td id="LC1586" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1587" class="blob-num js-line-number" data-line-number="1587"></td>
        <td id="LC1587" class="blob-code blob-code-inner js-file-line">							<span class="pl-k">if</span> (iter_s != pVehicle-&gt;m_SpeedProfile.<span class="pl-c1">begin</span>())</td>
      </tr>
      <tr>
        <td id="L1588" class="blob-num js-line-number" data-line-number="1588"></td>
        <td id="LC1588" class="blob-code blob-code-inner js-file-line">							{</td>
      </tr>
      <tr>
        <td id="L1589" class="blob-num js-line-number" data-line-number="1589"></td>
        <td id="LC1589" class="blob-code blob-code-inner js-file-line">								totalCO2 += element.<span class="pl-smi">CO2</span>;</td>
      </tr>
      <tr>
        <td id="L1590" class="blob-num js-line-number" data-line-number="1590"></td>
        <td id="LC1590" class="blob-code blob-code-inner js-file-line">								totalNOX += element.<span class="pl-smi">NOX</span>;</td>
      </tr>
      <tr>
        <td id="L1591" class="blob-num js-line-number" data-line-number="1591"></td>
        <td id="LC1591" class="blob-code blob-code-inner js-file-line">								totalCO += element.<span class="pl-smi">CO</span>;</td>
      </tr>
      <tr>
        <td id="L1592" class="blob-num js-line-number" data-line-number="1592"></td>
        <td id="LC1592" class="blob-code blob-code-inner js-file-line">								totalHC += element.<span class="pl-smi">HC</span>;</td>
      </tr>
      <tr>
        <td id="L1593" class="blob-num js-line-number" data-line-number="1593"></td>
        <td id="LC1593" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1594" class="blob-num js-line-number" data-line-number="1594"></td>
        <td id="LC1594" class="blob-code blob-code-inner js-file-line">								<span class="pl-c1">fprintf</span>(fpSampledEmissionsFile,<span class="pl-s"><span class="pl-pds">&quot;</span>,<span class="pl-c1">%5.2f</span>,<span class="pl-c1">%f</span>,<span class="pl-c1">%s</span>,<span class="pl-c1">%s</span>,<span class="pl-c1">%d</span>,<span class="pl-c1">%f</span>,<span class="pl-c1">%f</span>,<span class="pl-c1">%f</span>,<span class="pl-c1">%f</span>,<span class="pl-c1">%f</span><span class="pl-pds">&quot;</span></span>,</td>
      </tr>
      <tr>
        <td id="L1595" class="blob-num js-line-number" data-line-number="1595"></td>
        <td id="LC1595" class="blob-code blob-code-inner js-file-line">									element.<span class="pl-smi">Acceleration</span>,</td>
      </tr>
      <tr>
        <td id="L1596" class="blob-num js-line-number" data-line-number="1596"></td>
        <td id="LC1596" class="blob-code blob-code-inner js-file-line">									element.<span class="pl-smi">VSP</span>,</td>
      </tr>
      <tr>
        <td id="L1597" class="blob-num js-line-number" data-line-number="1597"></td>
        <td id="LC1597" class="blob-code blob-code-inner js-file-line">									element.<span class="pl-c1">GetVSPBinNoString</span>(),</td>
      </tr>
      <tr>
        <td id="L1598" class="blob-num js-line-number" data-line-number="1598"></td>
        <td id="LC1598" class="blob-code blob-code-inner js-file-line">									element.<span class="pl-c1">GetSpeedBinNoString</span>(),</td>
      </tr>
      <tr>
        <td id="L1599" class="blob-num js-line-number" data-line-number="1599"></td>
        <td id="LC1599" class="blob-code blob-code-inner js-file-line">									element.<span class="pl-smi">OperationMode</span>,</td>
      </tr>
      <tr>
        <td id="L1600" class="blob-num js-line-number" data-line-number="1600"></td>
        <td id="LC1600" class="blob-code blob-code-inner js-file-line">									element.<span class="pl-smi">Energy</span>,</td>
      </tr>
      <tr>
        <td id="L1601" class="blob-num js-line-number" data-line-number="1601"></td>
        <td id="LC1601" class="blob-code blob-code-inner js-file-line">									element.<span class="pl-smi">CO2</span>, </td>
      </tr>
      <tr>
        <td id="L1602" class="blob-num js-line-number" data-line-number="1602"></td>
        <td id="LC1602" class="blob-code blob-code-inner js-file-line">									element.<span class="pl-smi">NOX</span>, </td>
      </tr>
      <tr>
        <td id="L1603" class="blob-num js-line-number" data-line-number="1603"></td>
        <td id="LC1603" class="blob-code blob-code-inner js-file-line">									element.<span class="pl-smi">CO</span>,</td>
      </tr>
      <tr>
        <td id="L1604" class="blob-num js-line-number" data-line-number="1604"></td>
        <td id="LC1604" class="blob-code blob-code-inner js-file-line">									element.<span class="pl-smi">HC</span></td>
      </tr>
      <tr>
        <td id="L1605" class="blob-num js-line-number" data-line-number="1605"></td>
        <td id="LC1605" class="blob-code blob-code-inner js-file-line">									);</td>
      </tr>
      <tr>
        <td id="L1606" class="blob-num js-line-number" data-line-number="1606"></td>
        <td id="LC1606" class="blob-code blob-code-inner js-file-line">							}</td>
      </tr>
      <tr>
        <td id="L1607" class="blob-num js-line-number" data-line-number="1607"></td>
        <td id="LC1607" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1608" class="blob-num js-line-number" data-line-number="1608"></td>
        <td id="LC1608" class="blob-code blob-code-inner js-file-line">							AverageSpeed += element.<span class="pl-smi">Speed</span>;</td>
      </tr>
      <tr>
        <td id="L1609" class="blob-num js-line-number" data-line-number="1609"></td>
        <td id="LC1609" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1610" class="blob-num js-line-number" data-line-number="1610"></td>
        <td id="LC1610" class="blob-code blob-code-inner js-file-line">							<span class="pl-k">if</span> (speedProfileCounter + <span class="pl-c1">1</span> == speedProfileSize)</td>
      </tr>
      <tr>
        <td id="L1611" class="blob-num js-line-number" data-line-number="1611"></td>
        <td id="LC1611" class="blob-code blob-code-inner js-file-line">							{</td>
      </tr>
      <tr>
        <td id="L1612" class="blob-num js-line-number" data-line-number="1612"></td>
        <td id="LC1612" class="blob-code blob-code-inner js-file-line">								<span class="pl-c1">fprintf</span>(fpSampledCycleEmissionRateFile, <span class="pl-s"><span class="pl-pds">&quot;</span><span class="pl-c1">%d</span>,<span class="pl-c1">%d</span>,<span class="pl-c1">%d</span><span class="pl-pds">&quot;</span></span>, </td>
      </tr>
      <tr>
        <td id="L1613" class="blob-num js-line-number" data-line-number="1613"></td>
        <td id="LC1613" class="blob-code blob-code-inner js-file-line">									pVehicle-&gt;m_AgentID,</td>
      </tr>
      <tr>
        <td id="L1614" class="blob-num js-line-number" data-line-number="1614"></td>
        <td id="LC1614" class="blob-code blob-code-inner js-file-line">									pVehicle-&gt;m_VehicleType,</td>
      </tr>
      <tr>
        <td id="L1615" class="blob-num js-line-number" data-line-number="1615"></td>
        <td id="LC1615" class="blob-code blob-code-inner js-file-line">									pVehicle-&gt;m_Age</td>
      </tr>
      <tr>
        <td id="L1616" class="blob-num js-line-number" data-line-number="1616"></td>
        <td id="LC1616" class="blob-code blob-code-inner js-file-line">									);</td>
      </tr>
      <tr>
        <td id="L1617" class="blob-num js-line-number" data-line-number="1617"></td>
        <td id="LC1617" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1618" class="blob-num js-line-number" data-line-number="1618"></td>
        <td id="LC1618" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">float</span> average_speed =  AverageSpeed / speedProfileSize <span class="pl-c">/*pVehicle-&gt;m_Distance / (pVehicle-&gt;m_ArrivalTime - pVehicle-&gt;m_DepartureTime) * 60.0f*/</span>;</td>
      </tr>
      <tr>
        <td id="L1619" class="blob-num js-line-number" data-line-number="1619"></td>
        <td id="LC1619" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">float</span> base_average_speed;</td>
      </tr>
      <tr>
        <td id="L1620" class="blob-num js-line-number" data-line-number="1620"></td>
        <td id="LC1620" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">//Average speed for base cycle is 21.2 mph</span></td>
      </tr>
      <tr>
        <td id="L1621" class="blob-num js-line-number" data-line-number="1621"></td>
        <td id="LC1621" class="blob-code blob-code-inner js-file-line">								CCycleAverageEmissionFactor BaseCaseEmissionsFactors = CycleAverageEmissionFactorMatrix[pVehicle-&gt;m_VehicleType][pVehicle-&gt;m_Age];</td>
      </tr>
      <tr>
        <td id="L1622" class="blob-num js-line-number" data-line-number="1622"></td>
        <td id="LC1622" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">float</span> speed_ratio = <span class="pl-c1">21.2</span> / average_speed;</td>
      </tr>
      <tr>
        <td id="L1623" class="blob-num js-line-number" data-line-number="1623"></td>
        <td id="LC1623" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1624" class="blob-num js-line-number" data-line-number="1624"></td>
        <td id="LC1624" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">float</span> correctedCO2 = BaseCaseEmissionsFactors.<span class="pl-smi">emissionFactor_CO2</span> * (totalCO2 / speedProfileSize * <span class="pl-c1">3600</span>) / BaseCycleEmissionRate[pVehicle-&gt;m_VehicleType][pVehicle-&gt;m_Age][EMISSION_CO2] * speed_ratio;</td>
      </tr>
      <tr>
        <td id="L1625" class="blob-num js-line-number" data-line-number="1625"></td>
        <td id="LC1625" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">float</span> correctedNOX = BaseCaseEmissionsFactors.<span class="pl-smi">emissionFactor_NOX</span> * (totalNOX / speedProfileSize * <span class="pl-c1">3600</span>) / BaseCycleEmissionRate[pVehicle-&gt;m_VehicleType][pVehicle-&gt;m_Age][EMISSION_NOX] * speed_ratio;</td>
      </tr>
      <tr>
        <td id="L1626" class="blob-num js-line-number" data-line-number="1626"></td>
        <td id="LC1626" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">float</span> correctedCO = BaseCaseEmissionsFactors.<span class="pl-smi">emissionFactor_CO</span> * (totalCO / speedProfileSize * <span class="pl-c1">3600</span>) / BaseCycleEmissionRate[pVehicle-&gt;m_VehicleType][pVehicle-&gt;m_Age][EMISSION_CO] * speed_ratio;</td>
      </tr>
      <tr>
        <td id="L1627" class="blob-num js-line-number" data-line-number="1627"></td>
        <td id="LC1627" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">float</span> correctedHC = BaseCaseEmissionsFactors.<span class="pl-smi">emissionFactor_HC</span> * (totalHC / speedProfileSize * <span class="pl-c1">3600</span>) / BaseCycleEmissionRate[pVehicle-&gt;m_VehicleType][pVehicle-&gt;m_Age][EMISSION_HC] * speed_ratio;</td>
      </tr>
      <tr>
        <td id="L1628" class="blob-num js-line-number" data-line-number="1628"></td>
        <td id="LC1628" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1629" class="blob-num js-line-number" data-line-number="1629"></td>
        <td id="LC1629" class="blob-code blob-code-inner js-file-line">								<span class="pl-c1">fprintf</span>(fpSampledEmissionsFile, <span class="pl-s"><span class="pl-pds">&quot;</span>,<span class="pl-c1">%f</span>,<span class="pl-c1">%f</span>,<span class="pl-c1">%f</span>,<span class="pl-c1">%f</span>,<span class="pl-c1">%f</span><span class="pl-pds">&quot;</span></span>, average_speed, correctedCO2, correctedNOX, correctedCO, correctedHC);</td>
      </tr>
      <tr>
        <td id="L1630" class="blob-num js-line-number" data-line-number="1630"></td>
        <td id="LC1630" class="blob-code blob-code-inner js-file-line">								<span class="pl-c1">fprintf</span>(fpSampledCycleEmissionRateFile, <span class="pl-s"><span class="pl-pds">&quot;</span>,<span class="pl-c1">%f</span>,<span class="pl-c1">%f</span>,<span class="pl-c1">%f</span>,<span class="pl-c1">%f</span>,<span class="pl-c1">%f</span><span class="pl-pds">&quot;</span></span>, average_speed, correctedCO2, correctedNOX, correctedCO, correctedHC);</td>
      </tr>
      <tr>
        <td id="L1631" class="blob-num js-line-number" data-line-number="1631"></td>
        <td id="LC1631" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1632" class="blob-num js-line-number" data-line-number="1632"></td>
        <td id="LC1632" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1633" class="blob-num js-line-number" data-line-number="1633"></td>
        <td id="LC1633" class="blob-code blob-code-inner js-file-line">								<span class="pl-c1">fprintf</span>(fpSampledCycleEmissionRateFile,<span class="pl-s"><span class="pl-pds">&quot;</span><span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>);</td>
      </tr>
      <tr>
        <td id="L1634" class="blob-num js-line-number" data-line-number="1634"></td>
        <td id="LC1634" class="blob-code blob-code-inner js-file-line">							}</td>
      </tr>
      <tr>
        <td id="L1635" class="blob-num js-line-number" data-line-number="1635"></td>
        <td id="LC1635" class="blob-code blob-code-inner js-file-line">							<span class="pl-c1">fprintf</span>(fpSampledEmissionsFile,<span class="pl-s"><span class="pl-pds">&quot;</span><span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>);</td>
      </tr>
      <tr>
        <td id="L1636" class="blob-num js-line-number" data-line-number="1636"></td>
        <td id="LC1636" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1637" class="blob-num js-line-number" data-line-number="1637"></td>
        <td id="LC1637" class="blob-code blob-code-inner js-file-line">							prev_speed = element.<span class="pl-smi">Speed</span>;</td>
      </tr>
      <tr>
        <td id="L1638" class="blob-num js-line-number" data-line-number="1638"></td>
        <td id="LC1638" class="blob-code blob-code-inner js-file-line">							prev_prev_acceleration = prev_acceleration;</td>
      </tr>
      <tr>
        <td id="L1639" class="blob-num js-line-number" data-line-number="1639"></td>
        <td id="LC1639" class="blob-code blob-code-inner js-file-line">							prev_acceleration = element.<span class="pl-smi">Acceleration</span> / <span class="pl-c1">3</span>.<span class="pl-c1">28084f</span>;</td>
      </tr>
      <tr>
        <td id="L1640" class="blob-num js-line-number" data-line-number="1640"></td>
        <td id="LC1640" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L1641" class="blob-num js-line-number" data-line-number="1641"></td>
        <td id="LC1641" class="blob-code blob-code-inner js-file-line">					}</td>
      </tr>
      <tr>
        <td id="L1642" class="blob-num js-line-number" data-line-number="1642"></td>
        <td id="LC1642" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L1643" class="blob-num js-line-number" data-line-number="1643"></td>
        <td id="LC1643" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1644" class="blob-num js-line-number" data-line-number="1644"></td>
        <td id="LC1644" class="blob-code blob-code-inner js-file-line">				<span class="pl-c1">fprintf</span>(st2, <span class="pl-s"><span class="pl-pds">&quot;</span>Vehicle=,<span class="pl-c1">%d</span>,Type=,<span class="pl-c1">%d</span><span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>, pVehicle-&gt;m_AgentID,pVehicle-&gt;m_VehicleType);</td>
      </tr>
      <tr>
        <td id="L1645" class="blob-num js-line-number" data-line-number="1645"></td>
        <td id="LC1645" class="blob-code blob-code-inner js-file-line">				<span class="pl-c1">fprintf</span>(st2, <span class="pl-s"><span class="pl-pds">&quot;</span># of operating mode data points =,<span class="pl-c1">%d</span><span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>, pVehicle-&gt;m_OperatingModeCount.<span class="pl-c1">size</span>());</td>
      </tr>
      <tr>
        <td id="L1646" class="blob-num js-line-number" data-line-number="1646"></td>
        <td id="LC1646" class="blob-code blob-code-inner js-file-line">				<span class="pl-c1">fprintf</span>(st2, <span class="pl-s"><span class="pl-pds">&quot;</span>Energy:, <span class="pl-c1">%f</span><span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>, pVehicle-&gt;Energy);</td>
      </tr>
      <tr>
        <td id="L1647" class="blob-num js-line-number" data-line-number="1647"></td>
        <td id="LC1647" class="blob-code blob-code-inner js-file-line">				<span class="pl-c1">fprintf</span>(st2, <span class="pl-s"><span class="pl-pds">&quot;</span>CO2:, <span class="pl-c1">%f</span><span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>, pVehicle-&gt;CO2 );</td>
      </tr>
      <tr>
        <td id="L1648" class="blob-num js-line-number" data-line-number="1648"></td>
        <td id="LC1648" class="blob-code blob-code-inner js-file-line">				<span class="pl-c1">fprintf</span>(st2, <span class="pl-s"><span class="pl-pds">&quot;</span>CO:, <span class="pl-c1">%f</span><span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>, pVehicle-&gt;CO );</td>
      </tr>
      <tr>
        <td id="L1649" class="blob-num js-line-number" data-line-number="1649"></td>
        <td id="LC1649" class="blob-code blob-code-inner js-file-line">				<span class="pl-c1">fprintf</span>(st2, <span class="pl-s"><span class="pl-pds">&quot;</span>HC:, <span class="pl-c1">%f</span><span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>, pVehicle-&gt;HC );</td>
      </tr>
      <tr>
        <td id="L1650" class="blob-num js-line-number" data-line-number="1650"></td>
        <td id="LC1650" class="blob-code blob-code-inner js-file-line">				<span class="pl-c1">fprintf</span>(st2, <span class="pl-s"><span class="pl-pds">&quot;</span>NOX:, <span class="pl-c1">%f</span><span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>, pVehicle-&gt;NOX);</td>
      </tr>
      <tr>
        <td id="L1651" class="blob-num js-line-number" data-line-number="1651"></td>
        <td id="LC1651" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1652" class="blob-num js-line-number" data-line-number="1652"></td>
        <td id="LC1652" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">for</span>(std::map&lt;<span class="pl-k">int</span>, <span class="pl-k">int</span>&gt;::iterator iterOP  =  pVehicle-&gt;m_OperatingModeCount.<span class="pl-c1">begin</span>(); iterOP != pVehicle-&gt;m_OperatingModeCount.<span class="pl-c1">end</span> (); iterOP++)</td>
      </tr>
      <tr>
        <td id="L1653" class="blob-num js-line-number" data-line-number="1653"></td>
        <td id="LC1653" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L1654" class="blob-num js-line-number" data-line-number="1654"></td>
        <td id="LC1654" class="blob-code blob-code-inner js-file-line">					<span class="pl-c1">fprintf</span>(st2, <span class="pl-s"><span class="pl-pds">&quot;</span>op=,<span class="pl-c1">%d</span>,count=,<span class="pl-c1">%d</span><span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>, iterOP-&gt;first ,iterOP-&gt;second );</td>
      </tr>
      <tr>
        <td id="L1655" class="blob-num js-line-number" data-line-number="1655"></td>
        <td id="LC1655" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L1656" class="blob-num js-line-number" data-line-number="1656"></td>
        <td id="LC1656" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1657" class="blob-num js-line-number" data-line-number="1657"></td>
        <td id="LC1657" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">for</span>(std::map&lt;<span class="pl-k">int</span>, <span class="pl-k">int</span>&gt;::iterator iter_speed  =  pVehicle-&gt;m_SpeedCount.<span class="pl-c1">begin</span>(); iter_speed != pVehicle-&gt;m_SpeedCount.<span class="pl-c1">end</span> (); iter_speed++)</td>
      </tr>
      <tr>
        <td id="L1658" class="blob-num js-line-number" data-line-number="1658"></td>
        <td id="LC1658" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L1659" class="blob-num js-line-number" data-line-number="1659"></td>
        <td id="LC1659" class="blob-code blob-code-inner js-file-line">					<span class="pl-c1">fprintf</span>(st2, <span class="pl-s"><span class="pl-pds">&quot;</span>speed=,<span class="pl-c1">%d</span>,count=,<span class="pl-c1">%d</span><span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>, iter_speed-&gt;first ,iter_speed-&gt;second );</td>
      </tr>
      <tr>
        <td id="L1660" class="blob-num js-line-number" data-line-number="1660"></td>
        <td id="LC1660" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L1661" class="blob-num js-line-number" data-line-number="1661"></td>
        <td id="LC1661" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1662" class="blob-num js-line-number" data-line-number="1662"></td>
        <td id="LC1662" class="blob-code blob-code-inner js-file-line">				<span class="pl-c1">fprintf</span>(st2,<span class="pl-s"><span class="pl-pds">&quot;</span><span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>);</td>
      </tr>
      <tr>
        <td id="L1663" class="blob-num js-line-number" data-line-number="1663"></td>
        <td id="LC1663" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L1664" class="blob-num js-line-number" data-line-number="1664"></td>
        <td id="LC1664" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1665" class="blob-num js-line-number" data-line-number="1665"></td>
        <td id="LC1665" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (fpSampledEmissionsFile)</td>
      </tr>
      <tr>
        <td id="L1666" class="blob-num js-line-number" data-line-number="1666"></td>
        <td id="LC1666" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L1667" class="blob-num js-line-number" data-line-number="1667"></td>
        <td id="LC1667" class="blob-code blob-code-inner js-file-line">				<span class="pl-c1">fclose</span>(fpSampledEmissionsFile);</td>
      </tr>
      <tr>
        <td id="L1668" class="blob-num js-line-number" data-line-number="1668"></td>
        <td id="LC1668" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L1669" class="blob-num js-line-number" data-line-number="1669"></td>
        <td id="LC1669" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1670" class="blob-num js-line-number" data-line-number="1670"></td>
        <td id="LC1670" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (fpSampledCycleEmissionRateFile)</td>
      </tr>
      <tr>
        <td id="L1671" class="blob-num js-line-number" data-line-number="1671"></td>
        <td id="LC1671" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L1672" class="blob-num js-line-number" data-line-number="1672"></td>
        <td id="LC1672" class="blob-code blob-code-inner js-file-line">				<span class="pl-c1">fclose</span>(fpSampledCycleEmissionRateFile);</td>
      </tr>
      <tr>
        <td id="L1673" class="blob-num js-line-number" data-line-number="1673"></td>
        <td id="LC1673" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L1674" class="blob-num js-line-number" data-line-number="1674"></td>
        <td id="LC1674" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1675" class="blob-num js-line-number" data-line-number="1675"></td>
        <td id="LC1675" class="blob-code blob-code-inner js-file-line">			<span class="pl-c1">fclose</span>(st2);</td>
      </tr>
      <tr>
        <td id="L1676" class="blob-num js-line-number" data-line-number="1676"></td>
        <td id="LC1676" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1677" class="blob-num js-line-number" data-line-number="1677"></td>
        <td id="LC1677" class="blob-code blob-code-inner js-file-line">			cout &lt;&lt; numOfSamples &lt;&lt; <span class="pl-s"><span class="pl-pds">&quot;</span> vehicles are sampled to output_sampled_vehicle_operating_mode.csv.<span class="pl-pds">&quot;</span></span> &lt;&lt; endl;</td>
      </tr>
      <tr>
        <td id="L1678" class="blob-num js-line-number" data-line-number="1678"></td>
        <td id="LC1678" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L1679" class="blob-num js-line-number" data-line-number="1679"></td>
        <td id="LC1679" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1680" class="blob-num js-line-number" data-line-number="1680"></td>
        <td id="LC1680" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L1681" class="blob-num js-line-number" data-line-number="1681"></td>
        <td id="LC1681" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">endif</span> </td>
      </tr>
      <tr>
        <td id="L1682" class="blob-num js-line-number" data-line-number="1682"></td>
        <td id="LC1682" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L1683" class="blob-num js-line-number" data-line-number="1683"></td>
        <td id="LC1683" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1684" class="blob-num js-line-number" data-line-number="1684"></td>
        <td id="LC1684" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//void SmoothCumulativeCounts(float* countArray, int size)</span></td>
      </tr>
      <tr>
        <td id="L1685" class="blob-num js-line-number" data-line-number="1685"></td>
        <td id="LC1685" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//{</span></td>
      </tr>
      <tr>
        <td id="L1686" class="blob-num js-line-number" data-line-number="1686"></td>
        <td id="LC1686" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//    if (countArray != NULL &amp;&amp; size &gt; 0)</span></td>
      </tr>
      <tr>
        <td id="L1687" class="blob-num js-line-number" data-line-number="1687"></td>
        <td id="LC1687" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//    {</span></td>
      </tr>
      <tr>
        <td id="L1688" class="blob-num js-line-number" data-line-number="1688"></td>
        <td id="LC1688" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//        float value = countArray[0];</span></td>
      </tr>
      <tr>
        <td id="L1689" class="blob-num js-line-number" data-line-number="1689"></td>
        <td id="LC1689" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//        int start_idx = 0;</span></td>
      </tr>
      <tr>
        <td id="L1690" class="blob-num js-line-number" data-line-number="1690"></td>
        <td id="LC1690" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//        int end_index = 0;</span></td>
      </tr>
      <tr>
        <td id="L1691" class="blob-num js-line-number" data-line-number="1691"></td>
        <td id="LC1691" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//        for (int n = 1; n &lt; size; n++)</span></td>
      </tr>
      <tr>
        <td id="L1692" class="blob-num js-line-number" data-line-number="1692"></td>
        <td id="LC1692" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//        {</span></td>
      </tr>
      <tr>
        <td id="L1693" class="blob-num js-line-number" data-line-number="1693"></td>
        <td id="LC1693" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//            if (countArray[n] == value)</span></td>
      </tr>
      <tr>
        <td id="L1694" class="blob-num js-line-number" data-line-number="1694"></td>
        <td id="LC1694" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//            {</span></td>
      </tr>
      <tr>
        <td id="L1695" class="blob-num js-line-number" data-line-number="1695"></td>
        <td id="LC1695" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//                countArray[n] = 0;</span></td>
      </tr>
      <tr>
        <td id="L1696" class="blob-num js-line-number" data-line-number="1696"></td>
        <td id="LC1696" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//                end_index++;</span></td>
      </tr>
      <tr>
        <td id="L1697" class="blob-num js-line-number" data-line-number="1697"></td>
        <td id="LC1697" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//            }</span></td>
      </tr>
      <tr>
        <td id="L1698" class="blob-num js-line-number" data-line-number="1698"></td>
        <td id="LC1698" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//            else</span></td>
      </tr>
      <tr>
        <td id="L1699" class="blob-num js-line-number" data-line-number="1699"></td>
        <td id="LC1699" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//            {</span></td>
      </tr>
      <tr>
        <td id="L1700" class="blob-num js-line-number" data-line-number="1700"></td>
        <td id="LC1700" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//                if (start_idx &lt; end_index)</span></td>
      </tr>
      <tr>
        <td id="L1701" class="blob-num js-line-number" data-line-number="1701"></td>
        <td id="LC1701" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//                {</span></td>
      </tr>
      <tr>
        <td id="L1702" class="blob-num js-line-number" data-line-number="1702"></td>
        <td id="LC1702" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//                    float increment = (countArray[n] - value) / (end_index - start_idx + 1);</span></td>
      </tr>
      <tr>
        <td id="L1703" class="blob-num js-line-number" data-line-number="1703"></td>
        <td id="LC1703" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//                    for (int i = start_idx + 1; i &lt;= end_index; i++)</span></td>
      </tr>
      <tr>
        <td id="L1704" class="blob-num js-line-number" data-line-number="1704"></td>
        <td id="LC1704" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//                    {</span></td>
      </tr>
      <tr>
        <td id="L1705" class="blob-num js-line-number" data-line-number="1705"></td>
        <td id="LC1705" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//                        countArray[i] = countArray[i - 1] + increment;</span></td>
      </tr>
      <tr>
        <td id="L1706" class="blob-num js-line-number" data-line-number="1706"></td>
        <td id="LC1706" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//                    }</span></td>
      </tr>
      <tr>
        <td id="L1707" class="blob-num js-line-number" data-line-number="1707"></td>
        <td id="LC1707" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//                }</span></td>
      </tr>
      <tr>
        <td id="L1708" class="blob-num js-line-number" data-line-number="1708"></td>
        <td id="LC1708" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//                value = countArray[n];</span></td>
      </tr>
      <tr>
        <td id="L1709" class="blob-num js-line-number" data-line-number="1709"></td>
        <td id="LC1709" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//                start_idx = end_index = n;</span></td>
      </tr>
      <tr>
        <td id="L1710" class="blob-num js-line-number" data-line-number="1710"></td>
        <td id="LC1710" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//            }</span></td>
      </tr>
      <tr>
        <td id="L1711" class="blob-num js-line-number" data-line-number="1711"></td>
        <td id="LC1711" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//        }</span></td>
      </tr>
      <tr>
        <td id="L1712" class="blob-num js-line-number" data-line-number="1712"></td>
        <td id="LC1712" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//    }</span></td>
      </tr>
      <tr>
        <td id="L1713" class="blob-num js-line-number" data-line-number="1713"></td>
        <td id="LC1713" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//}</span></td>
      </tr>
      <tr>
        <td id="L1714" class="blob-num js-line-number" data-line-number="1714"></td>
        <td id="LC1714" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1715" class="blob-num js-line-number" data-line-number="1715"></td>
        <td id="LC1715" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//void DTALink::ThreeDetectorVehicleTrajectorySimulation()</span></td>
      </tr>
      <tr>
        <td id="L1716" class="blob-num js-line-number" data-line-number="1716"></td>
        <td id="LC1716" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//{</span></td>
      </tr>
      <tr>
        <td id="L1717" class="blob-num js-line-number" data-line-number="1717"></td>
        <td id="LC1717" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	int initialTime = -1;</span></td>
      </tr>
      <tr>
        <td id="L1718" class="blob-num js-line-number" data-line-number="1718"></td>
        <td id="LC1718" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	int endTime = -1;</span></td>
      </tr>
      <tr>
        <td id="L1719" class="blob-num js-line-number" data-line-number="1719"></td>
        <td id="LC1719" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1720" class="blob-num js-line-number" data-line-number="1720"></td>
        <td id="LC1720" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1721" class="blob-num js-line-number" data-line-number="1721"></td>
        <td id="LC1721" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	for (int LaneNo = 0; LaneNo &lt; m_OutflowNumLanes; LaneNo++)</span></td>
      </tr>
      <tr>
        <td id="L1722" class="blob-num js-line-number" data-line-number="1722"></td>
        <td id="LC1722" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	{</span></td>
      </tr>
      <tr>
        <td id="L1723" class="blob-num js-line-number" data-line-number="1723"></td>
        <td id="LC1723" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		sort(m_VehicleDataVector[LaneNo].LaneData.begin(), m_VehicleDataVector[LaneNo].LaneData.end(), vehicleCF_sort_function);</span></td>
      </tr>
      <tr>
        <td id="L1724" class="blob-num js-line-number" data-line-number="1724"></td>
        <td id="LC1724" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		initialTime = m_VehicleDataVector[LaneNo].LaneData[0].StartTime_in_SimulationInterval;</span></td>
      </tr>
      <tr>
        <td id="L1725" class="blob-num js-line-number" data-line-number="1725"></td>
        <td id="LC1725" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		endTime = m_VehicleDataVector[LaneNo].LaneData[m_VehicleDataVector[LaneNo].LaneData.size() -1].EndTime_in_SimulationInterval;</span></td>
      </tr>
      <tr>
        <td id="L1726" class="blob-num js-line-number" data-line-number="1726"></td>
        <td id="LC1726" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		int countArraySize = endTime - initialTime + 1;</span></td>
      </tr>
      <tr>
        <td id="L1727" class="blob-num js-line-number" data-line-number="1727"></td>
        <td id="LC1727" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		float* downstreamLaneCounts = new float[countArraySize];</span></td>
      </tr>
      <tr>
        <td id="L1728" class="blob-num js-line-number" data-line-number="1728"></td>
        <td id="LC1728" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		memset(downstreamLaneCounts, 0, sizeof(float) * countArraySize);</span></td>
      </tr>
      <tr>
        <td id="L1729" class="blob-num js-line-number" data-line-number="1729"></td>
        <td id="LC1729" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1730" class="blob-num js-line-number" data-line-number="1730"></td>
        <td id="LC1730" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		for(int v = 0; v &lt; m_VehicleDataVector[LaneNo].LaneData.size(); v++)  // first do loop: every vehicle</span></td>
      </tr>
      <tr>
        <td id="L1731" class="blob-num js-line-number" data-line-number="1731"></td>
        <td id="LC1731" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		{</span></td>
      </tr>
      <tr>
        <td id="L1732" class="blob-num js-line-number" data-line-number="1732"></td>
        <td id="LC1732" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			int start_time =  m_VehicleDataVector[LaneNo].LaneData[v].StartTime_in_SimulationInterval;</span></td>
      </tr>
      <tr>
        <td id="L1733" class="blob-num js-line-number" data-line-number="1733"></td>
        <td id="LC1733" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			int end_time =  m_VehicleDataVector[LaneNo].LaneData[v].EndTime_in_SimulationInterval;</span></td>
      </tr>
      <tr>
        <td id="L1734" class="blob-num js-line-number" data-line-number="1734"></td>
        <td id="LC1734" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			downstreamLaneCounts[end_time - initialTime]++;</span></td>
      </tr>
      <tr>
        <td id="L1735" class="blob-num js-line-number" data-line-number="1735"></td>
        <td id="LC1735" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		}</span></td>
      </tr>
      <tr>
        <td id="L1736" class="blob-num js-line-number" data-line-number="1736"></td>
        <td id="LC1736" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1737" class="blob-num js-line-number" data-line-number="1737"></td>
        <td id="LC1737" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		for (int i=1; i &lt; countArraySize; i++)</span></td>
      </tr>
      <tr>
        <td id="L1738" class="blob-num js-line-number" data-line-number="1738"></td>
        <td id="LC1738" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		{</span></td>
      </tr>
      <tr>
        <td id="L1739" class="blob-num js-line-number" data-line-number="1739"></td>
        <td id="LC1739" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			downstreamLaneCounts[i] += downstreamLaneCounts[i -1];</span></td>
      </tr>
      <tr>
        <td id="L1740" class="blob-num js-line-number" data-line-number="1740"></td>
        <td id="LC1740" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		}</span></td>
      </tr>
      <tr>
        <td id="L1741" class="blob-num js-line-number" data-line-number="1741"></td>
        <td id="LC1741" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1742" class="blob-num js-line-number" data-line-number="1742"></td>
        <td id="LC1742" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		SmoothCumulativeCounts(downstreamLaneCounts, countArraySize);</span></td>
      </tr>
      <tr>
        <td id="L1743" class="blob-num js-line-number" data-line-number="1743"></td>
        <td id="LC1743" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1744" class="blob-num js-line-number" data-line-number="1744"></td>
        <td id="LC1744" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1745" class="blob-num js-line-number" data-line-number="1745"></td>
        <td id="LC1745" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		for(int v = 0; v &lt; m_VehicleDataVector[LaneNo].LaneData.size(); v++)  // first do loop: every vehicle</span></td>
      </tr>
      <tr>
        <td id="L1746" class="blob-num js-line-number" data-line-number="1746"></td>
        <td id="LC1746" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		{</span></td>
      </tr>
      <tr>
        <td id="L1747" class="blob-num js-line-number" data-line-number="1747"></td>
        <td id="LC1747" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			int start_time =  m_VehicleDataVector[LaneNo].LaneData[v].StartTime_in_SimulationInterval;</span></td>
      </tr>
      <tr>
        <td id="L1748" class="blob-num js-line-number" data-line-number="1748"></td>
        <td id="LC1748" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			int end_time =  m_VehicleDataVector[LaneNo].LaneData[v].EndTime_in_SimulationInterval;</span></td>
      </tr>
      <tr>
        <td id="L1749" class="blob-num js-line-number" data-line-number="1749"></td>
        <td id="LC1749" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			</span></td>
      </tr>
      <tr>
        <td id="L1750" class="blob-num js-line-number" data-line-number="1750"></td>
        <td id="LC1750" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			float link_length_in_meter = m_Length * 1609.344f;</span></td>
      </tr>
      <tr>
        <td id="L1751" class="blob-num js-line-number" data-line-number="1751"></td>
        <td id="LC1751" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			float position = 0.0f;</span></td>
      </tr>
      <tr>
        <td id="L1752" class="blob-num js-line-number" data-line-number="1752"></td>
        <td id="LC1752" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			int upstreamCountNum = v;</span></td>
      </tr>
      <tr>
        <td id="L1753" class="blob-num js-line-number" data-line-number="1753"></td>
        <td id="LC1753" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1754" class="blob-num js-line-number" data-line-number="1754"></td>
        <td id="LC1754" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			DTAVehicle* pVehicle  = g_VehicleMap[m_VehicleDataVector[LaneNo].LaneData[v].VehicleID];</span></td>
      </tr>
      <tr>
        <td id="L1755" class="blob-num js-line-number" data-line-number="1755"></td>
        <td id="LC1755" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			pVehicle-&gt;m_PrevSpeed = 0.0f;</span></td>
      </tr>
      <tr>
        <td id="L1756" class="blob-num js-line-number" data-line-number="1756"></td>
        <td id="LC1756" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			float prevPosition = 0.0f;</span></td>
      </tr>
      <tr>
        <td id="L1757" class="blob-num js-line-number" data-line-number="1757"></td>
        <td id="LC1757" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			float prev_acceleration = -99999.0f; //acceleration at t-1</span></td>
      </tr>
      <tr>
        <td id="L1758" class="blob-num js-line-number" data-line-number="1758"></td>
        <td id="LC1758" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			float prev_prev_acceleration = -99999.0f; //acceleration at t-2</span></td>
      </tr>
      <tr>
        <td id="L1759" class="blob-num js-line-number" data-line-number="1759"></td>
        <td id="LC1759" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1760" class="blob-num js-line-number" data-line-number="1760"></td>
        <td id="LC1760" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			for(int t = 1; t &lt; end_time - start_time; t+= 1) 				</span></td>
      </tr>
      <tr>
        <td id="L1761" class="blob-num js-line-number" data-line-number="1761"></td>
        <td id="LC1761" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			{</span></td>
      </tr>
      <tr>
        <td id="L1762" class="blob-num js-line-number" data-line-number="1762"></td>
        <td id="LC1762" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//				if (position &lt; link_length_in_meter)</span></td>
      </tr>
      <tr>
        <td id="L1763" class="blob-num js-line-number" data-line-number="1763"></td>
        <td id="LC1763" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//				{</span></td>
      </tr>
      <tr>
        <td id="L1764" class="blob-num js-line-number" data-line-number="1764"></td>
        <td id="LC1764" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					int bwtt = (int)((link_length_in_meter - position) / (this-&gt;m_BackwardWaveSpeed * 1609.34f / 3600.0f)) * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;</span></td>
      </tr>
      <tr>
        <td id="L1765" class="blob-num js-line-number" data-line-number="1765"></td>
        <td id="LC1765" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					int t_at_downstream = t - bwtt;</span></td>
      </tr>
      <tr>
        <td id="L1766" class="blob-num js-line-number" data-line-number="1766"></td>
        <td id="LC1766" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					if (t_at_downstream &lt; 0)</span></td>
      </tr>
      <tr>
        <td id="L1767" class="blob-num js-line-number" data-line-number="1767"></td>
        <td id="LC1767" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					{</span></td>
      </tr>
      <tr>
        <td id="L1768" class="blob-num js-line-number" data-line-number="1768"></td>
        <td id="LC1768" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						position = min(link_length_in_meter, position + this-&gt;m_SpeedLimit * 1609.34f / 3600.0f / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND);						</span></td>
      </tr>
      <tr>
        <td id="L1769" class="blob-num js-line-number" data-line-number="1769"></td>
        <td id="LC1769" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					}</span></td>
      </tr>
      <tr>
        <td id="L1770" class="blob-num js-line-number" data-line-number="1770"></td>
        <td id="LC1770" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					else</span></td>
      </tr>
      <tr>
        <td id="L1771" class="blob-num js-line-number" data-line-number="1771"></td>
        <td id="LC1771" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					{</span></td>
      </tr>
      <tr>
        <td id="L1772" class="blob-num js-line-number" data-line-number="1772"></td>
        <td id="LC1772" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						if (upstreamCountNum &lt;= downstreamLaneCounts[t_at_downstream] + this-&gt;m_KJam * (link_length_in_meter - position) * 0.000621371f)</span></td>
      </tr>
      <tr>
        <td id="L1773" class="blob-num js-line-number" data-line-number="1773"></td>
        <td id="LC1773" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						{</span></td>
      </tr>
      <tr>
        <td id="L1774" class="blob-num js-line-number" data-line-number="1774"></td>
        <td id="LC1774" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//							position = min(link_length_in_meter, position + this-&gt;m_SpeedLimit * 1609.34f / 3600.0f / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND);							</span></td>
      </tr>
      <tr>
        <td id="L1775" class="blob-num js-line-number" data-line-number="1775"></td>
        <td id="LC1775" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						}</span></td>
      </tr>
      <tr>
        <td id="L1776" class="blob-num js-line-number" data-line-number="1776"></td>
        <td id="LC1776" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					}</span></td>
      </tr>
      <tr>
        <td id="L1777" class="blob-num js-line-number" data-line-number="1777"></td>
        <td id="LC1777" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//				}</span></td>
      </tr>
      <tr>
        <td id="L1778" class="blob-num js-line-number" data-line-number="1778"></td>
        <td id="LC1778" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//				else</span></td>
      </tr>
      <tr>
        <td id="L1779" class="blob-num js-line-number" data-line-number="1779"></td>
        <td id="LC1779" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//				{</span></td>
      </tr>
      <tr>
        <td id="L1780" class="blob-num js-line-number" data-line-number="1780"></td>
        <td id="LC1780" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					break;</span></td>
      </tr>
      <tr>
        <td id="L1781" class="blob-num js-line-number" data-line-number="1781"></td>
        <td id="LC1781" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//				}</span></td>
      </tr>
      <tr>
        <td id="L1782" class="blob-num js-line-number" data-line-number="1782"></td>
        <td id="LC1782" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1783" class="blob-num js-line-number" data-line-number="1783"></td>
        <td id="LC1783" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//				if (t % NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND == 0)</span></td>
      </tr>
      <tr>
        <td id="L1784" class="blob-num js-line-number" data-line-number="1784"></td>
        <td id="LC1784" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//				{</span></td>
      </tr>
      <tr>
        <td id="L1785" class="blob-num js-line-number" data-line-number="1785"></td>
        <td id="LC1785" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1786" class="blob-num js-line-number" data-line-number="1786"></td>
        <td id="LC1786" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					float SpeedBySecond = position - prevPosition;					</span></td>
      </tr>
      <tr>
        <td id="L1787" class="blob-num js-line-number" data-line-number="1787"></td>
        <td id="LC1787" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					float acceleration = SpeedBySecond - pVehicle-&gt;m_PrevSpeed;</span></td>
      </tr>
      <tr>
        <td id="L1788" class="blob-num js-line-number" data-line-number="1788"></td>
        <td id="LC1788" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1789" class="blob-num js-line-number" data-line-number="1789"></td>
        <td id="LC1789" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					float vsp = 0;</span></td>
      </tr>
      <tr>
        <td id="L1790" class="blob-num js-line-number" data-line-number="1790"></td>
        <td id="LC1790" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					int OperatingMode = ComputeOperatingModeFromSpeed(vsp,SpeedBySecond, acceleration,0,pVehicle-&gt;m_VehicleType);</span></td>
      </tr>
      <tr>
        <td id="L1791" class="blob-num js-line-number" data-line-number="1791"></td>
        <td id="LC1791" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1792" class="blob-num js-line-number" data-line-number="1792"></td>
        <td id="LC1792" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					if (acceleration &lt;= -2)</span></td>
      </tr>
      <tr>
        <td id="L1793" class="blob-num js-line-number" data-line-number="1793"></td>
        <td id="LC1793" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					{</span></td>
      </tr>
      <tr>
        <td id="L1794" class="blob-num js-line-number" data-line-number="1794"></td>
        <td id="LC1794" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						OperatingMode = 0;</span></td>
      </tr>
      <tr>
        <td id="L1795" class="blob-num js-line-number" data-line-number="1795"></td>
        <td id="LC1795" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					}</span></td>
      </tr>
      <tr>
        <td id="L1796" class="blob-num js-line-number" data-line-number="1796"></td>
        <td id="LC1796" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					else</span></td>
      </tr>
      <tr>
        <td id="L1797" class="blob-num js-line-number" data-line-number="1797"></td>
        <td id="LC1797" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					{</span></td>
      </tr>
      <tr>
        <td id="L1798" class="blob-num js-line-number" data-line-number="1798"></td>
        <td id="LC1798" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						if (prev_prev_acceleration &lt; -1 &amp;&amp; prev_acceleration &lt; -1 &amp;&amp; acceleration &lt; -1)</span></td>
      </tr>
      <tr>
        <td id="L1799" class="blob-num js-line-number" data-line-number="1799"></td>
        <td id="LC1799" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						{</span></td>
      </tr>
      <tr>
        <td id="L1800" class="blob-num js-line-number" data-line-number="1800"></td>
        <td id="LC1800" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//							if (prev_prev_acceleration != -99999.0f || prev_acceleration != -99999.0f)</span></td>
      </tr>
      <tr>
        <td id="L1801" class="blob-num js-line-number" data-line-number="1801"></td>
        <td id="LC1801" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//							{</span></td>
      </tr>
      <tr>
        <td id="L1802" class="blob-num js-line-number" data-line-number="1802"></td>
        <td id="LC1802" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//								OperatingMode = 0;</span></td>
      </tr>
      <tr>
        <td id="L1803" class="blob-num js-line-number" data-line-number="1803"></td>
        <td id="LC1803" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//							}</span></td>
      </tr>
      <tr>
        <td id="L1804" class="blob-num js-line-number" data-line-number="1804"></td>
        <td id="LC1804" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						}</span></td>
      </tr>
      <tr>
        <td id="L1805" class="blob-num js-line-number" data-line-number="1805"></td>
        <td id="LC1805" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					}</span></td>
      </tr>
      <tr>
        <td id="L1806" class="blob-num js-line-number" data-line-number="1806"></td>
        <td id="LC1806" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1807" class="blob-num js-line-number" data-line-number="1807"></td>
        <td id="LC1807" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					if ((SpeedBySecond / 0.44704f) &gt;= -1.0f &amp;&amp; (SpeedBySecond / 0.44704f) &lt; 1.0f)</span></td>
      </tr>
      <tr>
        <td id="L1808" class="blob-num js-line-number" data-line-number="1808"></td>
        <td id="LC1808" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					{</span></td>
      </tr>
      <tr>
        <td id="L1809" class="blob-num js-line-number" data-line-number="1809"></td>
        <td id="LC1809" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						OperatingMode = 1;</span></td>
      </tr>
      <tr>
        <td id="L1810" class="blob-num js-line-number" data-line-number="1810"></td>
        <td id="LC1810" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					}</span></td>
      </tr>
      <tr>
        <td id="L1811" class="blob-num js-line-number" data-line-number="1811"></td>
        <td id="LC1811" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1812" class="blob-num js-line-number" data-line-number="1812"></td>
        <td id="LC1812" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					pVehicle-&gt;m_OperatingModeCount[OperatingMode]++;</span></td>
      </tr>
      <tr>
        <td id="L1813" class="blob-num js-line-number" data-line-number="1813"></td>
        <td id="LC1813" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					int integer_speed = SpeedBySecond / 0.44704f;  // convert meter per second to mile per hour</span></td>
      </tr>
      <tr>
        <td id="L1814" class="blob-num js-line-number" data-line-number="1814"></td>
        <td id="LC1814" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					pVehicle-&gt;m_SpeedCount[integer_speed]++;</span></td>
      </tr>
      <tr>
        <td id="L1815" class="blob-num js-line-number" data-line-number="1815"></td>
        <td id="LC1815" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1816" class="blob-num js-line-number" data-line-number="1816"></td>
        <td id="LC1816" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1817" class="blob-num js-line-number" data-line-number="1817"></td>
        <td id="LC1817" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					pVehicle-&gt;m_PrevSpeed  = SpeedBySecond;</span></td>
      </tr>
      <tr>
        <td id="L1818" class="blob-num js-line-number" data-line-number="1818"></td>
        <td id="LC1818" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1819" class="blob-num js-line-number" data-line-number="1819"></td>
        <td id="LC1819" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					int vehicle_type = pVehicle-&gt;m_VehicleType;</span></td>
      </tr>
      <tr>
        <td id="L1820" class="blob-num js-line-number" data-line-number="1820"></td>
        <td id="LC1820" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1821" class="blob-num js-line-number" data-line-number="1821"></td>
        <td id="LC1821" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					int age = pVehicle-&gt;m_Age ;</span></td>
      </tr>
      <tr>
        <td id="L1822" class="blob-num js-line-number" data-line-number="1822"></td>
        <td id="LC1822" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1823" class="blob-num js-line-number" data-line-number="1823"></td>
        <td id="LC1823" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					int time_in_min = t / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND / 60;</span></td>
      </tr>
      <tr>
        <td id="L1824" class="blob-num js-line-number" data-line-number="1824"></td>
        <td id="LC1824" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1825" class="blob-num js-line-number" data-line-number="1825"></td>
        <td id="LC1825" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					if(EmissionRateData[vehicle_type][OperatingMode][age].bInitialized == false)</span></td>
      </tr>
      <tr>
        <td id="L1826" class="blob-num js-line-number" data-line-number="1826"></td>
        <td id="LC1826" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					{</span></td>
      </tr>
      <tr>
        <td id="L1827" class="blob-num js-line-number" data-line-number="1827"></td>
        <td id="LC1827" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						cout &lt;&lt; &quot;Emission rate data are not available for vehicle type = &quot; &lt;&lt;  vehicle_type </span></td>
      </tr>
      <tr>
        <td id="L1828" class="blob-num js-line-number" data-line-number="1828"></td>
        <td id="LC1828" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//							&lt;&lt; &quot;, operating mode = &quot; &lt;&lt; OperatingMode </span></td>
      </tr>
      <tr>
        <td id="L1829" class="blob-num js-line-number" data-line-number="1829"></td>
        <td id="LC1829" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//							&lt;&lt; &quot; and age = &quot; &lt;&lt; age &lt;&lt; endl;</span></td>
      </tr>
      <tr>
        <td id="L1830" class="blob-num js-line-number" data-line-number="1830"></td>
        <td id="LC1830" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						g_ProgramStop();</span></td>
      </tr>
      <tr>
        <td id="L1831" class="blob-num js-line-number" data-line-number="1831"></td>
        <td id="LC1831" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					}</span></td>
      </tr>
      <tr>
        <td id="L1832" class="blob-num js-line-number" data-line-number="1832"></td>
        <td id="LC1832" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1833" class="blob-num js-line-number" data-line-number="1833"></td>
        <td id="LC1833" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					float Energy = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_TotalEnergy/3600.0f;</span></td>
      </tr>
      <tr>
        <td id="L1834" class="blob-num js-line-number" data-line-number="1834"></td>
        <td id="LC1834" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					float CO2 = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_CO2/3600.0f;</span></td>
      </tr>
      <tr>
        <td id="L1835" class="blob-num js-line-number" data-line-number="1835"></td>
        <td id="LC1835" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					float NOX = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_NOX/3600.0f;</span></td>
      </tr>
      <tr>
        <td id="L1836" class="blob-num js-line-number" data-line-number="1836"></td>
        <td id="LC1836" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					float CO = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_CO/3600.0f;</span></td>
      </tr>
      <tr>
        <td id="L1837" class="blob-num js-line-number" data-line-number="1837"></td>
        <td id="LC1837" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					float HC = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_HC/3600.0f;</span></td>
      </tr>
      <tr>
        <td id="L1838" class="blob-num js-line-number" data-line-number="1838"></td>
        <td id="LC1838" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1839" class="blob-num js-line-number" data-line-number="1839"></td>
        <td id="LC1839" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].Energy+= Energy;</span></td>
      </tr>
      <tr>
        <td id="L1840" class="blob-num js-line-number" data-line-number="1840"></td>
        <td id="LC1840" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].CO2+= CO2;</span></td>
      </tr>
      <tr>
        <td id="L1841" class="blob-num js-line-number" data-line-number="1841"></td>
        <td id="LC1841" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].NOX+= NOX;</span></td>
      </tr>
      <tr>
        <td id="L1842" class="blob-num js-line-number" data-line-number="1842"></td>
        <td id="LC1842" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].CO+= CO;</span></td>
      </tr>
      <tr>
        <td id="L1843" class="blob-num js-line-number" data-line-number="1843"></td>
        <td id="LC1843" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].HC+= HC;</span></td>
      </tr>
      <tr>
        <td id="L1844" class="blob-num js-line-number" data-line-number="1844"></td>
        <td id="LC1844" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1845" class="blob-num js-line-number" data-line-number="1845"></td>
        <td id="LC1845" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					if (pVehicle-&gt;m_AgentID  ==  g_TargetVehicleID_OutputSecondBySecondEmissionData || </span></td>
      </tr>
      <tr>
        <td id="L1846" class="blob-num js-line-number" data-line-number="1846"></td>
        <td id="LC1846" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						(g_OutputSecondBySecondEmissionData &amp;&amp; pVehicle-&gt;m_DepartureTime &gt;= g_start_departure_time_for_output_second_by_second_emission_data </span></td>
      </tr>
      <tr>
        <td id="L1847" class="blob-num js-line-number" data-line-number="1847"></td>
        <td id="LC1847" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						&amp;&amp; pVehicle-&gt;m_DepartureTime &lt;= g_end_departure_time_for_output_second_by_second_emission_data</span></td>
      </tr>
      <tr>
        <td id="L1848" class="blob-num js-line-number" data-line-number="1848"></td>
        <td id="LC1848" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						&amp;&amp; pVehicle-&gt;m_bDetailedEmissionOutput))  // record data</span></td>
      </tr>
      <tr>
        <td id="L1849" class="blob-num js-line-number" data-line-number="1849"></td>
        <td id="LC1849" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					{</span></td>
      </tr>
      <tr>
        <td id="L1850" class="blob-num js-line-number" data-line-number="1850"></td>
        <td id="LC1850" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						VehicleSpeedProfileData element;</span></td>
      </tr>
      <tr>
        <td id="L1851" class="blob-num js-line-number" data-line-number="1851"></td>
        <td id="LC1851" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						element.FromNodeNumber = this-&gt;m_FromNodeNumber ;</span></td>
      </tr>
      <tr>
        <td id="L1852" class="blob-num js-line-number" data-line-number="1852"></td>
        <td id="LC1852" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						element.ToNodeNumber  = this-&gt;m_ToNodeNumber ;</span></td>
      </tr>
      <tr>
        <td id="L1853" class="blob-num js-line-number" data-line-number="1853"></td>
        <td id="LC1853" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						element.TimeOnThisLinkInSecond = (t-start_time)/NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;</span></td>
      </tr>
      <tr>
        <td id="L1854" class="blob-num js-line-number" data-line-number="1854"></td>
        <td id="LC1854" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						element.Speed = SpeedBySecond / 0.44704f; // km/h to mph</span></td>
      </tr>
      <tr>
        <td id="L1855" class="blob-num js-line-number" data-line-number="1855"></td>
        <td id="LC1855" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						element.Acceleration = acceleration * 3.28084; //meter/sec^2 to feet/sec^2</span></td>
      </tr>
      <tr>
        <td id="L1856" class="blob-num js-line-number" data-line-number="1856"></td>
        <td id="LC1856" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						element.OperationMode = OperatingMode;</span></td>
      </tr>
      <tr>
        <td id="L1857" class="blob-num js-line-number" data-line-number="1857"></td>
        <td id="LC1857" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						element.VSP  = vsp;</span></td>
      </tr>
      <tr>
        <td id="L1858" class="blob-num js-line-number" data-line-number="1858"></td>
        <td id="LC1858" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						element.SpeedBinNo = GetSpeedBinNo(element.Speed);</span></td>
      </tr>
      <tr>
        <td id="L1859" class="blob-num js-line-number" data-line-number="1859"></td>
        <td id="LC1859" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						element.VSPBinNo =  GetVSPBinNo(vsp, element.Speed);</span></td>
      </tr>
      <tr>
        <td id="L1860" class="blob-num js-line-number" data-line-number="1860"></td>
        <td id="LC1860" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						element.Energy = Energy;</span></td>
      </tr>
      <tr>
        <td id="L1861" class="blob-num js-line-number" data-line-number="1861"></td>
        <td id="LC1861" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						element.CO2 = CO2;</span></td>
      </tr>
      <tr>
        <td id="L1862" class="blob-num js-line-number" data-line-number="1862"></td>
        <td id="LC1862" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						element.NOX = NOX;</span></td>
      </tr>
      <tr>
        <td id="L1863" class="blob-num js-line-number" data-line-number="1863"></td>
        <td id="LC1863" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						element.CO = CO;</span></td>
      </tr>
      <tr>
        <td id="L1864" class="blob-num js-line-number" data-line-number="1864"></td>
        <td id="LC1864" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						element.HC = HC;</span></td>
      </tr>
      <tr>
        <td id="L1865" class="blob-num js-line-number" data-line-number="1865"></td>
        <td id="LC1865" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1866" class="blob-num js-line-number" data-line-number="1866"></td>
        <td id="LC1866" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//						pVehicle-&gt;m_SpeedProfile[t] = element;</span></td>
      </tr>
      <tr>
        <td id="L1867" class="blob-num js-line-number" data-line-number="1867"></td>
        <td id="LC1867" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					}</span></td>
      </tr>
      <tr>
        <td id="L1868" class="blob-num js-line-number" data-line-number="1868"></td>
        <td id="LC1868" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1869" class="blob-num js-line-number" data-line-number="1869"></td>
        <td id="LC1869" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					prevPosition = position;</span></td>
      </tr>
      <tr>
        <td id="L1870" class="blob-num js-line-number" data-line-number="1870"></td>
        <td id="LC1870" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					prev_prev_acceleration = prev_acceleration;</span></td>
      </tr>
      <tr>
        <td id="L1871" class="blob-num js-line-number" data-line-number="1871"></td>
        <td id="LC1871" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//					prev_acceleration = acceleration * 0.44704f;</span></td>
      </tr>
      <tr>
        <td id="L1872" class="blob-num js-line-number" data-line-number="1872"></td>
        <td id="LC1872" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//				}				</span></td>
      </tr>
      <tr>
        <td id="L1873" class="blob-num js-line-number" data-line-number="1873"></td>
        <td id="LC1873" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//			}</span></td>
      </tr>
      <tr>
        <td id="L1874" class="blob-num js-line-number" data-line-number="1874"></td>
        <td id="LC1874" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		}</span></td>
      </tr>
      <tr>
        <td id="L1875" class="blob-num js-line-number" data-line-number="1875"></td>
        <td id="LC1875" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1876" class="blob-num js-line-number" data-line-number="1876"></td>
        <td id="LC1876" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1877" class="blob-num js-line-number" data-line-number="1877"></td>
        <td id="LC1877" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//		delete [] downstreamLaneCounts;</span></td>
      </tr>
      <tr>
        <td id="L1878" class="blob-num js-line-number" data-line-number="1878"></td>
        <td id="LC1878" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//	}</span></td>
      </tr>
      <tr>
        <td id="L1879" class="blob-num js-line-number" data-line-number="1879"></td>
        <td id="LC1879" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//}</span></td>
      </tr>
      <tr>
        <td id="L1880" class="blob-num js-line-number" data-line-number="1880"></td>
        <td id="LC1880" class="blob-code blob-code-inner js-file-line"><span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L1881" class="blob-num js-line-number" data-line-number="1881"></td>
        <td id="LC1881" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1882" class="blob-num js-line-number" data-line-number="1882"></td>
        <td id="LC1882" class="blob-code blob-code-inner js-file-line"><span class="pl-k">void</span> <span class="pl-en">DTALink::ComputeVSP</span>()  <span class="pl-c">// VSP: vehicle specific power</span></td>
      </tr>
      <tr>
        <td id="L1883" class="blob-num js-line-number" data-line-number="1883"></td>
        <td id="LC1883" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L1884" class="blob-num js-line-number" data-line-number="1884"></td>
        <td id="LC1884" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">ifdef</span> _large_memory_usage</td>
      </tr>
      <tr>
        <td id="L1885" class="blob-num js-line-number" data-line-number="1885"></td>
        <td id="LC1885" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1886" class="blob-num js-line-number" data-line-number="1886"></td>
        <td id="LC1886" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">const</span> <span class="pl-k">int</span> Max_Time_Step_In_A_Cycle = <span class="pl-c1">100</span>;</td>
      </tr>
      <tr>
        <td id="L1887" class="blob-num js-line-number" data-line-number="1887"></td>
        <td id="LC1887" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">const</span> <span class="pl-k">int</span> Max_Simulation_Steps = <span class="pl-c1">1800</span> * <span class="pl-c1">60</span> * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;</td>
      </tr>
      <tr>
        <td id="L1888" class="blob-num js-line-number" data-line-number="1888"></td>
        <td id="LC1888" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">static</span> <span class="pl-k">float</span> positionBuffer1[Max_Simulation_Steps];</td>
      </tr>
      <tr>
        <td id="L1889" class="blob-num js-line-number" data-line-number="1889"></td>
        <td id="LC1889" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">static</span> <span class="pl-k">float</span> positionsBuffer2[Max_Simulation_Steps];	</td>
      </tr>
      <tr>
        <td id="L1890" class="blob-num js-line-number" data-line-number="1890"></td>
        <td id="LC1890" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> *pLeaderPositions = positionBuffer1;</td>
      </tr>
      <tr>
        <td id="L1891" class="blob-num js-line-number" data-line-number="1891"></td>
        <td id="LC1891" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> *pFollowerPositions = positionsBuffer2;</td>
      </tr>
      <tr>
        <td id="L1892" class="blob-num js-line-number" data-line-number="1892"></td>
        <td id="LC1892" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1893" class="blob-num js-line-number" data-line-number="1893"></td>
        <td id="LC1893" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//FILE* output = NULL;</span></td>
      </tr>
      <tr>
        <td id="L1894" class="blob-num js-line-number" data-line-number="1894"></td>
        <td id="LC1894" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1895" class="blob-num js-line-number" data-line-number="1895"></td>
        <td id="LC1895" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//if (this-&gt;m_FromNodeNumber == 2)</span></td>
      </tr>
      <tr>
        <td id="L1896" class="blob-num js-line-number" data-line-number="1896"></td>
        <td id="LC1896" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//{</span></td>
      </tr>
      <tr>
        <td id="L1897" class="blob-num js-line-number" data-line-number="1897"></td>
        <td id="LC1897" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//	fopen_s(&amp;output,&quot;trajectory.csv&quot;,&quot;w&quot;);</span></td>
      </tr>
      <tr>
        <td id="L1898" class="blob-num js-line-number" data-line-number="1898"></td>
        <td id="LC1898" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//}</span></td>
      </tr>
      <tr>
        <td id="L1899" class="blob-num js-line-number" data-line-number="1899"></td>
        <td id="LC1899" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1900" class="blob-num js-line-number" data-line-number="1900"></td>
        <td id="LC1900" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//#pragma omp parallel for </span></td>
      </tr>
      <tr>
        <td id="L1901" class="blob-num js-line-number" data-line-number="1901"></td>
        <td id="LC1901" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">for</span>(<span class="pl-k">int</span> LaneNo = <span class="pl-c1">0</span>; LaneNo &lt; m_OutflowNumLanes; LaneNo++)  <span class="pl-c">// for each lane </span></td>
      </tr>
      <tr>
        <td id="L1902" class="blob-num js-line-number" data-line-number="1902"></td>
        <td id="LC1902" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L1903" class="blob-num js-line-number" data-line-number="1903"></td>
        <td id="LC1903" class="blob-code blob-code-inner js-file-line">		<span class="pl-c">// step 1: sort vehicles arrival times</span></td>
      </tr>
      <tr>
        <td id="L1904" class="blob-num js-line-number" data-line-number="1904"></td>
        <td id="LC1904" class="blob-code blob-code-inner js-file-line">		<span class="pl-c1">sort</span>(m_VehicleDataVector[LaneNo].<span class="pl-smi">LaneData</span>.<span class="pl-c1">begin</span>(), m_VehicleDataVector[LaneNo].<span class="pl-smi">LaneData</span>.<span class="pl-c1">end</span>(), vehicleCF_sort_function);</td>
      </tr>
      <tr>
        <td id="L1905" class="blob-num js-line-number" data-line-number="1905"></td>
        <td id="LC1905" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1906" class="blob-num js-line-number" data-line-number="1906"></td>
        <td id="LC1906" class="blob-code blob-code-inner js-file-line">		<span class="pl-c">// step 2: car following simulation</span></td>
      </tr>
      <tr>
        <td id="L1907" class="blob-num js-line-number" data-line-number="1907"></td>
        <td id="LC1907" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">float</span> link_length_in_meter = m_Length * <span class="pl-c1">1609</span>.<span class="pl-c1">344f</span>; <span class="pl-c">//1609.344f: mile to meters</span></td>
      </tr>
      <tr>
        <td id="L1908" class="blob-num js-line-number" data-line-number="1908"></td>
        <td id="LC1908" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1909" class="blob-num js-line-number" data-line-number="1909"></td>
        <td id="LC1909" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">int</span> leaderInTime = -<span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L1910" class="blob-num js-line-number" data-line-number="1910"></td>
        <td id="LC1910" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">int</span> leaderOutTime = -<span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L1911" class="blob-num js-line-number" data-line-number="1911"></td>
        <td id="LC1911" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1912" class="blob-num js-line-number" data-line-number="1912"></td>
        <td id="LC1912" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">for</span>(<span class="pl-k">int</span> v = <span class="pl-c1">0</span>; v &lt; m_VehicleDataVector[LaneNo].<span class="pl-smi">LaneData</span>.<span class="pl-c1">size</span>(); v++)  <span class="pl-c">// first do loop: every vehicle</span></td>
      </tr>
      <tr>
        <td id="L1913" class="blob-num js-line-number" data-line-number="1913"></td>
        <td id="LC1913" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L1914" class="blob-num js-line-number" data-line-number="1914"></td>
        <td id="LC1914" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">int</span> start_time =  m_VehicleDataVector[LaneNo].<span class="pl-smi">LaneData</span>[v].<span class="pl-smi">StartTime_in_SimulationInterval</span>;</td>
      </tr>
      <tr>
        <td id="L1915" class="blob-num js-line-number" data-line-number="1915"></td>
        <td id="LC1915" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">int</span> end_time =  m_VehicleDataVector[LaneNo].<span class="pl-smi">LaneData</span>[v].<span class="pl-smi">EndTime_in_SimulationInterval</span>;</td>
      </tr>
      <tr>
        <td id="L1916" class="blob-num js-line-number" data-line-number="1916"></td>
        <td id="LC1916" class="blob-code blob-code-inner js-file-line">			DTAVehicle* pVehicle = g_VehicleMap[m_VehicleDataVector[LaneNo].<span class="pl-smi">LaneData</span>[v].<span class="pl-smi">VehicleID</span>];</td>
      </tr>
      <tr>
        <td id="L1917" class="blob-num js-line-number" data-line-number="1917"></td>
        <td id="LC1917" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">// second do loop: start_time to end time</span></td>
      </tr>
      <tr>
        <td id="L1918" class="blob-num js-line-number" data-line-number="1918"></td>
        <td id="LC1918" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">for</span>(<span class="pl-k">int</span> t = start_time; t &lt;= end_time; t+=<span class="pl-c1">1</span>) 				</td>
      </tr>
      <tr>
        <td id="L1919" class="blob-num js-line-number" data-line-number="1919"></td>
        <td id="LC1919" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L1920" class="blob-num js-line-number" data-line-number="1920"></td>
        <td id="LC1920" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">//calculate free-flow position</span></td>
      </tr>
      <tr>
        <td id="L1921" class="blob-num js-line-number" data-line-number="1921"></td>
        <td id="LC1921" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">//xiF(t) = xi(t-τ) + vf(τ)</span></td>
      </tr>
      <tr>
        <td id="L1922" class="blob-num js-line-number" data-line-number="1922"></td>
        <td id="LC1922" class="blob-code blob-code-inner js-file-line">				pFollowerPositions[t - start_time] = <span class="pl-c1">0</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L1923" class="blob-num js-line-number" data-line-number="1923"></td>
        <td id="LC1923" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1924" class="blob-num js-line-number" data-line-number="1924"></td>
        <td id="LC1924" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span>(t &gt; start_time)</td>
      </tr>
      <tr>
        <td id="L1925" class="blob-num js-line-number" data-line-number="1925"></td>
        <td id="LC1925" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L1926" class="blob-num js-line-number" data-line-number="1926"></td>
        <td id="LC1926" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">if</span> (v == <span class="pl-c1">0</span>)</td>
      </tr>
      <tr>
        <td id="L1927" class="blob-num js-line-number" data-line-number="1927"></td>
        <td id="LC1927" class="blob-code blob-code-inner js-file-line">					{</td>
      </tr>
      <tr>
        <td id="L1928" class="blob-num js-line-number" data-line-number="1928"></td>
        <td id="LC1928" class="blob-code blob-code-inner js-file-line">						pFollowerPositions[t - start_time] = <span class="pl-c1">max</span>(<span class="pl-c1">0</span>, <span class="pl-c1">min</span>(link_length_in_meter, pFollowerPositions[t - start_time - <span class="pl-c1">1</span>] +  m_VehicleDataVector[LaneNo].<span class="pl-smi">LaneData</span>[v].<span class="pl-smi">FreeflowDistance_per_SimulationInterval</span>));</td>
      </tr>
      <tr>
        <td id="L1929" class="blob-num js-line-number" data-line-number="1929"></td>
        <td id="LC1929" class="blob-code blob-code-inner js-file-line">					}</td>
      </tr>
      <tr>
        <td id="L1930" class="blob-num js-line-number" data-line-number="1930"></td>
        <td id="LC1930" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L1931" class="blob-num js-line-number" data-line-number="1931"></td>
        <td id="LC1931" class="blob-code blob-code-inner js-file-line">					{</td>
      </tr>
      <tr>
        <td id="L1932" class="blob-num js-line-number" data-line-number="1932"></td>
        <td id="LC1932" class="blob-code blob-code-inner js-file-line">						<span class="pl-c">//calculate congested position if it is not the first vehicle</span></td>
      </tr>
      <tr>
        <td id="L1933" class="blob-num js-line-number" data-line-number="1933"></td>
        <td id="LC1933" class="blob-code blob-code-inner js-file-line">						<span class="pl-c">//xiC(t) = xi-1(t-τ) - δ</span></td>
      </tr>
      <tr>
        <td id="L1934" class="blob-num js-line-number" data-line-number="1934"></td>
        <td id="LC1934" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">int</span> time_t_minus_tau = t - m_VehicleDataVector[LaneNo].<span class="pl-smi">LaneData</span>[v].<span class="pl-smi">TimeLag_in_SimulationInterval</span>; <span class="pl-c">// need to convert time in second to time in simulation time interval</span></td>
      </tr>
      <tr>
        <td id="L1935" class="blob-num js-line-number" data-line-number="1935"></td>
        <td id="LC1935" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">if</span> (time_t_minus_tau &lt; leaderInTime)</td>
      </tr>
      <tr>
        <td id="L1936" class="blob-num js-line-number" data-line-number="1936"></td>
        <td id="LC1936" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L1937" class="blob-num js-line-number" data-line-number="1937"></td>
        <td id="LC1937" class="blob-code blob-code-inner js-file-line">							pFollowerPositions[t - start_time] = <span class="pl-c1">0</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L1938" class="blob-num js-line-number" data-line-number="1938"></td>
        <td id="LC1938" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L1939" class="blob-num js-line-number" data-line-number="1939"></td>
        <td id="LC1939" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L1940" class="blob-num js-line-number" data-line-number="1940"></td>
        <td id="LC1940" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L1941" class="blob-num js-line-number" data-line-number="1941"></td>
        <td id="LC1941" class="blob-code blob-code-inner js-file-line">							<span class="pl-k">if</span>(time_t_minus_tau &gt;= <span class="pl-c1">0</span> &amp;&amp; time_t_minus_tau &lt;= leaderOutTime)  <span class="pl-c">// the leader has not reached destination yet</span></td>
      </tr>
      <tr>
        <td id="L1942" class="blob-num js-line-number" data-line-number="1942"></td>
        <td id="LC1942" class="blob-code blob-code-inner js-file-line">							{</td>
      </tr>
      <tr>
        <td id="L1943" class="blob-num js-line-number" data-line-number="1943"></td>
        <td id="LC1943" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">// vehicle v-1: previous car</span></td>
      </tr>
      <tr>
        <td id="L1944" class="blob-num js-line-number" data-line-number="1944"></td>
        <td id="LC1944" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">//float CongestedDistance = VechileDistanceAry[v-1][time_t_minus_tau%Max_Time_Step_In_A_Cycle] - m_VehicleDataVector[LaneNo].LaneData[v].CriticalSpacing_in_meter;</span></td>
      </tr>
      <tr>
        <td id="L1945" class="blob-num js-line-number" data-line-number="1945"></td>
        <td id="LC1945" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">float</span> CongestedDistance = <span class="pl-c1">max</span>(<span class="pl-c1">0</span>, pLeaderPositions[time_t_minus_tau - leaderInTime] - m_VehicleDataVector[LaneNo].<span class="pl-smi">LaneData</span>[v].<span class="pl-smi">CriticalSpacing_in_meter</span>);</td>
      </tr>
      <tr>
        <td id="L1946" class="blob-num js-line-number" data-line-number="1946"></td>
        <td id="LC1946" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">float</span> potentialDistance = <span class="pl-c1">min</span>(link_length_in_meter, pFollowerPositions[t - start_time - <span class="pl-c1">1</span>] + m_VehicleDataVector[LaneNo].<span class="pl-smi">LaneData</span>[v].<span class="pl-smi">FreeflowDistance_per_SimulationInterval</span>);</td>
      </tr>
      <tr>
        <td id="L1947" class="blob-num js-line-number" data-line-number="1947"></td>
        <td id="LC1947" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">// xi(t) = min(xAF(t), xAC(t))</span></td>
      </tr>
      <tr>
        <td id="L1948" class="blob-num js-line-number" data-line-number="1948"></td>
        <td id="LC1948" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">//if (FollowerPositions[t % Max_Simulation_Steps] &gt; CongestedDistance /*&amp;&amp; CongestedDistance &gt;= FollowerPositions[(t-1) % Max_Simulation_Steps]*/)</span></td>
      </tr>
      <tr>
        <td id="L1949" class="blob-num js-line-number" data-line-number="1949"></td>
        <td id="LC1949" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">//{</span></td>
      </tr>
      <tr>
        <td id="L1950" class="blob-num js-line-number" data-line-number="1950"></td>
        <td id="LC1950" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">//	FollowerPositions[t % Max_Simulation_Steps] = min(CongestedDistance, FollowerPositions[(t-1) % Max_Simulation_Steps]);</span></td>
      </tr>
      <tr>
        <td id="L1951" class="blob-num js-line-number" data-line-number="1951"></td>
        <td id="LC1951" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">//}</span></td>
      </tr>
      <tr>
        <td id="L1952" class="blob-num js-line-number" data-line-number="1952"></td>
        <td id="LC1952" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1953" class="blob-num js-line-number" data-line-number="1953"></td>
        <td id="LC1953" class="blob-code blob-code-inner js-file-line">								pFollowerPositions[t - start_time] = <span class="pl-c1">max</span>(pFollowerPositions[t - start_time-<span class="pl-c1">1</span>], <span class="pl-c1">min</span>(CongestedDistance, potentialDistance, link_length_in_meter)); </td>
      </tr>
      <tr>
        <td id="L1954" class="blob-num js-line-number" data-line-number="1954"></td>
        <td id="LC1954" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">//next step distance should be no less than the current distnance</span></td>
      </tr>
      <tr>
        <td id="L1955" class="blob-num js-line-number" data-line-number="1955"></td>
        <td id="LC1955" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">// next step distance should be min of (congestioned distance due to backward wave, and free-flow distance)</span></td>
      </tr>
      <tr>
        <td id="L1956" class="blob-num js-line-number" data-line-number="1956"></td>
        <td id="LC1956" class="blob-code blob-code-inner js-file-line">							}</td>
      </tr>
      <tr>
        <td id="L1957" class="blob-num js-line-number" data-line-number="1957"></td>
        <td id="LC1957" class="blob-code blob-code-inner js-file-line">							<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L1958" class="blob-num js-line-number" data-line-number="1958"></td>
        <td id="LC1958" class="blob-code blob-code-inner js-file-line">							{</td>
      </tr>
      <tr>
        <td id="L1959" class="blob-num js-line-number" data-line-number="1959"></td>
        <td id="LC1959" class="blob-code blob-code-inner js-file-line">								pFollowerPositions[t - start_time] = <span class="pl-c1">min</span>(link_length_in_meter, pFollowerPositions[t - start_time - <span class="pl-c1">1</span>] +  m_VehicleDataVector[LaneNo].<span class="pl-smi">LaneData</span>[v].<span class="pl-smi">FreeflowDistance_per_SimulationInterval</span>);</td>
      </tr>
      <tr>
        <td id="L1960" class="blob-num js-line-number" data-line-number="1960"></td>
        <td id="LC1960" class="blob-code blob-code-inner js-file-line">							}</td>
      </tr>
      <tr>
        <td id="L1961" class="blob-num js-line-number" data-line-number="1961"></td>
        <td id="LC1961" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L1962" class="blob-num js-line-number" data-line-number="1962"></td>
        <td id="LC1962" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1963" class="blob-num js-line-number" data-line-number="1963"></td>
        <td id="LC1963" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">if</span> (t == end_time)</td>
      </tr>
      <tr>
        <td id="L1964" class="blob-num js-line-number" data-line-number="1964"></td>
        <td id="LC1964" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L1965" class="blob-num js-line-number" data-line-number="1965"></td>
        <td id="LC1965" class="blob-code blob-code-inner js-file-line">							pFollowerPositions[t - start_time] = link_length_in_meter;</td>
      </tr>
      <tr>
        <td id="L1966" class="blob-num js-line-number" data-line-number="1966"></td>
        <td id="LC1966" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L1967" class="blob-num js-line-number" data-line-number="1967"></td>
        <td id="LC1967" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">if</span> (pVehicle-&gt;m_AgentID == g_TargetVehicleID_OutputSecondBySecondEmissionData)</td>
      </tr>
      <tr>
        <td id="L1968" class="blob-num js-line-number" data-line-number="1968"></td>
        <td id="LC1968" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L1969" class="blob-num js-line-number" data-line-number="1969"></td>
        <td id="LC1969" class="blob-code blob-code-inner js-file-line">							<span class="pl-k">if</span> (t != start_time)</td>
      </tr>
      <tr>
        <td id="L1970" class="blob-num js-line-number" data-line-number="1970"></td>
        <td id="LC1970" class="blob-code blob-code-inner js-file-line">							{</td>
      </tr>
      <tr>
        <td id="L1971" class="blob-num js-line-number" data-line-number="1971"></td>
        <td id="LC1971" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">int</span> second_from_time0 = t / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;</td>
      </tr>
      <tr>
        <td id="L1972" class="blob-num js-line-number" data-line-number="1972"></td>
        <td id="LC1972" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">int</span> hour = second_from_time0 / <span class="pl-c1">3600</span>;</td>
      </tr>
      <tr>
        <td id="L1973" class="blob-num js-line-number" data-line-number="1973"></td>
        <td id="LC1973" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">int</span> min = (second_from_time0 - hour * <span class="pl-c1">3600</span> ) / <span class="pl-c1">60</span>;</td>
      </tr>
      <tr>
        <td id="L1974" class="blob-num js-line-number" data-line-number="1974"></td>
        <td id="LC1974" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">int</span> second = (second_from_time0 - hour * <span class="pl-c1">3600</span>- min * <span class="pl-c1">60</span> );</td>
      </tr>
      <tr>
        <td id="L1975" class="blob-num js-line-number" data-line-number="1975"></td>
        <td id="LC1975" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1976" class="blob-num js-line-number" data-line-number="1976"></td>
        <td id="LC1976" class="blob-code blob-code-inner js-file-line">								<span class="pl-c1">ASSERT</span>(second &gt;= <span class="pl-c1">0</span>);</td>
      </tr>
      <tr>
        <td id="L1977" class="blob-num js-line-number" data-line-number="1977"></td>
        <td id="LC1977" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">double</span> moving_dist =  pFollowerPositions[t - start_time] - pFollowerPositions[t - start_time - <span class="pl-c1">1</span>];</td>
      </tr>
      <tr>
        <td id="L1978" class="blob-num js-line-number" data-line-number="1978"></td>
        <td id="LC1978" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">double</span> moving_speed = moving_dist * <span class="pl-c1">3600</span> * <span class="pl-c1">0.000621371</span>; <span class="pl-c">// moving distance from sec, to hour, and distance converted to mile</span></td>
      </tr>
      <tr>
        <td id="L1979" class="blob-num js-line-number" data-line-number="1979"></td>
        <td id="LC1979" class="blob-code blob-code-inner js-file-line">								<span class="pl-c1">_proxy_emission_log</span>(<span class="pl-c1">0</span>, pVehicle-&gt;m_AgentID, <span class="pl-s"><span class="pl-pds">&quot;</span><span class="pl-c1">%d</span>-&gt; <span class="pl-c1">%d</span>, lane:<span class="pl-c1">%d</span>, t:<span class="pl-c1">%d</span> (<span class="pl-c1">%02d</span>:<span class="pl-c1">%02d</span>:<span class="pl-c1">%02d</span>), dist:<span class="pl-c1">%5.2f</span>, moving_dist:<span class="pl-c1">%5.2f</span> (<span class="pl-c1">%5.3f</span> mph)<span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>, <span class="pl-v">this</span>-&gt;m_FromNodeNumber, <span class="pl-v">this</span>-&gt;m_ToNodeNumber,</td>
      </tr>
      <tr>
        <td id="L1980" class="blob-num js-line-number" data-line-number="1980"></td>
        <td id="LC1980" class="blob-code blob-code-inner js-file-line">									LaneNo, t, hour, min,second,</td>
      </tr>
      <tr>
        <td id="L1981" class="blob-num js-line-number" data-line-number="1981"></td>
        <td id="LC1981" class="blob-code blob-code-inner js-file-line">									pFollowerPositions[t - start_time], moving_dist, moving_speed);</td>
      </tr>
      <tr>
        <td id="L1982" class="blob-num js-line-number" data-line-number="1982"></td>
        <td id="LC1982" class="blob-code blob-code-inner js-file-line">							}</td>
      </tr>
      <tr>
        <td id="L1983" class="blob-num js-line-number" data-line-number="1983"></td>
        <td id="LC1983" class="blob-code blob-code-inner js-file-line">							<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L1984" class="blob-num js-line-number" data-line-number="1984"></td>
        <td id="LC1984" class="blob-code blob-code-inner js-file-line">							{</td>
      </tr>
      <tr>
        <td id="L1985" class="blob-num js-line-number" data-line-number="1985"></td>
        <td id="LC1985" class="blob-code blob-code-inner js-file-line">								<span class="pl-c1">_proxy_emission_log</span>(<span class="pl-c1">0</span>, pVehicle-&gt;m_AgentID, <span class="pl-s"><span class="pl-pds">&quot;</span><span class="pl-c1">%d</span>-&gt; <span class="pl-c1">%d</span>, lane:<span class="pl-c1">%d</span>, t:<span class="pl-c1">%d</span>, dist: <span class="pl-c1">%5.2f</span>, moving_dist: <span class="pl-c1">%5.2f</span> (0 mph)<span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>, <span class="pl-v">this</span>-&gt;m_FromNodeNumber, <span class="pl-v">this</span>-&gt;m_ToNodeNumber,</td>
      </tr>
      <tr>
        <td id="L1986" class="blob-num js-line-number" data-line-number="1986"></td>
        <td id="LC1986" class="blob-code blob-code-inner js-file-line">									LaneNo, t, pFollowerPositions[t - start_time], <span class="pl-c1">0</span>.<span class="pl-c1">0f</span>);</td>
      </tr>
      <tr>
        <td id="L1987" class="blob-num js-line-number" data-line-number="1987"></td>
        <td id="LC1987" class="blob-code blob-code-inner js-file-line">							}</td>
      </tr>
      <tr>
        <td id="L1988" class="blob-num js-line-number" data-line-number="1988"></td>
        <td id="LC1988" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L1989" class="blob-num js-line-number" data-line-number="1989"></td>
        <td id="LC1989" class="blob-code blob-code-inner js-file-line">					}</td>
      </tr>
      <tr>
        <td id="L1990" class="blob-num js-line-number" data-line-number="1990"></td>
        <td id="LC1990" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L1991" class="blob-num js-line-number" data-line-number="1991"></td>
        <td id="LC1991" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1992" class="blob-num js-line-number" data-line-number="1992"></td>
        <td id="LC1992" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1993" class="blob-num js-line-number" data-line-number="1993"></td>
        <td id="LC1993" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1994" class="blob-num js-line-number" data-line-number="1994"></td>
        <td id="LC1994" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1995" class="blob-num js-line-number" data-line-number="1995"></td>
        <td id="LC1995" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">//(v &gt;= 0 &amp;&amp; v &lt;= 400 &amp;&amp; LaneNo == 0)</span></td>
      </tr>
      <tr>
        <td id="L1996" class="blob-num js-line-number" data-line-number="1996"></td>
        <td id="LC1996" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">//{</span></td>
      </tr>
      <tr>
        <td id="L1997" class="blob-num js-line-number" data-line-number="1997"></td>
        <td id="LC1997" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">//	DTAVehicle* pVehicle  = g_VehicleMap[m_VehicleDataVector[LaneNo].LaneData[v].VehicleID];</span></td>
      </tr>
      <tr>
        <td id="L1998" class="blob-num js-line-number" data-line-number="1998"></td>
        <td id="LC1998" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L1999" class="blob-num js-line-number" data-line-number="1999"></td>
        <td id="LC1999" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">//	if (output != NULL)</span></td>
      </tr>
      <tr>
        <td id="L2000" class="blob-num js-line-number" data-line-number="2000"></td>
        <td id="LC2000" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">//	{</span></td>
      </tr>
      <tr>
        <td id="L2001" class="blob-num js-line-number" data-line-number="2001"></td>
        <td id="LC2001" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">//		if (t != start_time)</span></td>
      </tr>
      <tr>
        <td id="L2002" class="blob-num js-line-number" data-line-number="2002"></td>
        <td id="LC2002" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">//		{</span></td>
      </tr>
      <tr>
        <td id="L2003" class="blob-num js-line-number" data-line-number="2003"></td>
        <td id="LC2003" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">//			fprintf(output,&quot;%d, %d, %d, %d, %5.2f,%5.2f\n&quot;,v, pVehicle-&gt;m_AgentID, LaneNo, t, pFollowerPositions[t - start_time], pFollowerPositions[t - start_time] - pFollowerPositions[t- start_time - 1]);</span></td>
      </tr>
      <tr>
        <td id="L2004" class="blob-num js-line-number" data-line-number="2004"></td>
        <td id="LC2004" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">//		}</span></td>
      </tr>
      <tr>
        <td id="L2005" class="blob-num js-line-number" data-line-number="2005"></td>
        <td id="LC2005" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">//		else</span></td>
      </tr>
      <tr>
        <td id="L2006" class="blob-num js-line-number" data-line-number="2006"></td>
        <td id="LC2006" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">//		{</span></td>
      </tr>
      <tr>
        <td id="L2007" class="blob-num js-line-number" data-line-number="2007"></td>
        <td id="LC2007" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">//			fprintf(output,&quot;%d, %d, %d, %d,%5.2f,%5.2f\n&quot;,v, pVehicle-&gt;m_AgentID, LaneNo, t, pFollowerPositions[t - start_time], 0.0f);</span></td>
      </tr>
      <tr>
        <td id="L2008" class="blob-num js-line-number" data-line-number="2008"></td>
        <td id="LC2008" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">//		}</span></td>
      </tr>
      <tr>
        <td id="L2009" class="blob-num js-line-number" data-line-number="2009"></td>
        <td id="LC2009" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">//	}</span></td>
      </tr>
      <tr>
        <td id="L2010" class="blob-num js-line-number" data-line-number="2010"></td>
        <td id="LC2010" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">//}</span></td>
      </tr>
      <tr>
        <td id="L2011" class="blob-num js-line-number" data-line-number="2011"></td>
        <td id="LC2011" class="blob-code blob-code-inner js-file-line">			}  <span class="pl-c">// for each time step</span></td>
      </tr>
      <tr>
        <td id="L2012" class="blob-num js-line-number" data-line-number="2012"></td>
        <td id="LC2012" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2013" class="blob-num js-line-number" data-line-number="2013"></td>
        <td id="LC2013" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (g_EmissionSmoothVehicleTrajectory)</td>
      </tr>
      <tr>
        <td id="L2014" class="blob-num js-line-number" data-line-number="2014"></td>
        <td id="LC2014" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L2015" class="blob-num js-line-number" data-line-number="2015"></td>
        <td id="LC2015" class="blob-code blob-code-inner js-file-line">				<span class="pl-c1">MovingAverage</span>(pFollowerPositions, end_time - start_time + <span class="pl-c1">1</span>);</td>
      </tr>
      <tr>
        <td id="L2016" class="blob-num js-line-number" data-line-number="2016"></td>
        <td id="LC2016" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L2017" class="blob-num js-line-number" data-line-number="2017"></td>
        <td id="LC2017" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2018" class="blob-num js-line-number" data-line-number="2018"></td>
        <td id="LC2018" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">float</span> prev_acceleration = -<span class="pl-c1">99999</span>.<span class="pl-c1">0f</span>; <span class="pl-c">//acceleration at t-1</span></td>
      </tr>
      <tr>
        <td id="L2019" class="blob-num js-line-number" data-line-number="2019"></td>
        <td id="LC2019" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">float</span> prev_prev_acceleration = -<span class="pl-c1">99999</span>.<span class="pl-c1">0f</span>; <span class="pl-c">//acceleration at t-2</span></td>
      </tr>
      <tr>
        <td id="L2020" class="blob-num js-line-number" data-line-number="2020"></td>
        <td id="LC2020" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">for</span> (<span class="pl-k">int</span> t = start_time; t &lt;= end_time; t += NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND)</td>
      </tr>
      <tr>
        <td id="L2021" class="blob-num js-line-number" data-line-number="2021"></td>
        <td id="LC2021" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L2022" class="blob-num js-line-number" data-line-number="2022"></td>
        <td id="LC2022" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">// output speed per second</span></td>
      </tr>
      <tr>
        <td id="L2023" class="blob-num js-line-number" data-line-number="2023"></td>
        <td id="LC2023" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">// for active vehicles (with positive speed or positive distance</span></td>
      </tr>
      <tr>
        <td id="L2024" class="blob-num js-line-number" data-line-number="2024"></td>
        <td id="LC2024" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">float</span> SpeedBySecond = m_SpeedLimit * <span class="pl-c1">0</span>.<span class="pl-c1">44704f</span>; <span class="pl-c">// 1 mph = 0.44704 meters per second</span></td>
      </tr>
      <tr>
        <td id="L2025" class="blob-num js-line-number" data-line-number="2025"></td>
        <td id="LC2025" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2026" class="blob-num js-line-number" data-line-number="2026"></td>
        <td id="LC2026" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (t == start_time) SpeedBySecond = <span class="pl-c1">0</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L2027" class="blob-num js-line-number" data-line-number="2027"></td>
        <td id="LC2027" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2028" class="blob-num js-line-number" data-line-number="2028"></td>
        <td id="LC2028" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span>(t &gt;= start_time + NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND)  <span class="pl-c">// not the first second</span></td>
      </tr>
      <tr>
        <td id="L2029" class="blob-num js-line-number" data-line-number="2029"></td>
        <td id="LC2029" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L2030" class="blob-num js-line-number" data-line-number="2030"></td>
        <td id="LC2030" class="blob-code blob-code-inner js-file-line">					<span class="pl-c">//SpeedBySecond = (VechileDistanceAry[v][t%Max_Time_Step_In_A_Cycle] - VechileDistanceAry[v][max(0,t-NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND)%Max_Time_Step_In_A_Cycle]);						</span></td>
      </tr>
      <tr>
        <td id="L2031" class="blob-num js-line-number" data-line-number="2031"></td>
        <td id="LC2031" class="blob-code blob-code-inner js-file-line">					SpeedBySecond = <span class="pl-c1">min</span>(m_SpeedLimit * <span class="pl-c1">0</span>.<span class="pl-c1">44704f</span>,</td>
      </tr>
      <tr>
        <td id="L2032" class="blob-num js-line-number" data-line-number="2032"></td>
        <td id="LC2032" class="blob-code blob-code-inner js-file-line">						<span class="pl-c1">max</span>(<span class="pl-c1">0</span>, pFollowerPositions[t - start_time] - pFollowerPositions[t - start_time - NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND]));</td>
      </tr>
      <tr>
        <td id="L2033" class="blob-num js-line-number" data-line-number="2033"></td>
        <td id="LC2033" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L2034" class="blob-num js-line-number" data-line-number="2034"></td>
        <td id="LC2034" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2035" class="blob-num js-line-number" data-line-number="2035"></td>
        <td id="LC2035" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (t + NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND &gt;= end_time)</td>
      </tr>
      <tr>
        <td id="L2036" class="blob-num js-line-number" data-line-number="2036"></td>
        <td id="LC2036" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L2037" class="blob-num js-line-number" data-line-number="2037"></td>
        <td id="LC2037" class="blob-code blob-code-inner js-file-line">					SpeedBySecond = m_SpeedLimit * <span class="pl-c1">0</span>.<span class="pl-c1">44704f</span>;</td>
      </tr>
      <tr>
        <td id="L2038" class="blob-num js-line-number" data-line-number="2038"></td>
        <td id="LC2038" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L2039" class="blob-num js-line-number" data-line-number="2039"></td>
        <td id="LC2039" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2040" class="blob-num js-line-number" data-line-number="2040"></td>
        <td id="LC2040" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">// different lanes have different vehicle numbers, so we should not have openMP conflicts here</span></td>
      </tr>
      <tr>
        <td id="L2041" class="blob-num js-line-number" data-line-number="2041"></td>
        <td id="LC2041" class="blob-code blob-code-inner js-file-line">				DTAVehicle* pVehicle  = g_VehicleMap[m_VehicleDataVector[LaneNo].<span class="pl-smi">LaneData</span>[v].<span class="pl-smi">VehicleID</span>];</td>
      </tr>
      <tr>
        <td id="L2042" class="blob-num js-line-number" data-line-number="2042"></td>
        <td id="LC2042" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">float</span> acceleration = SpeedBySecond - pVehicle-&gt;m_PrevSpeed;</td>
      </tr>
      <tr>
        <td id="L2043" class="blob-num js-line-number" data-line-number="2043"></td>
        <td id="LC2043" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2044" class="blob-num js-line-number" data-line-number="2044"></td>
        <td id="LC2044" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">float</span> vsp = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L2045" class="blob-num js-line-number" data-line-number="2045"></td>
        <td id="LC2045" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">int</span> OperatingMode = <span class="pl-c1">ComputeOperatingModeFromSpeed</span>(vsp,SpeedBySecond, acceleration,<span class="pl-c1">0</span>,pVehicle-&gt;m_VehicleType);</td>
      </tr>
      <tr>
        <td id="L2046" class="blob-num js-line-number" data-line-number="2046"></td>
        <td id="LC2046" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2047" class="blob-num js-line-number" data-line-number="2047"></td>
        <td id="LC2047" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (acceleration &lt;= -<span class="pl-c1">2</span>)</td>
      </tr>
      <tr>
        <td id="L2048" class="blob-num js-line-number" data-line-number="2048"></td>
        <td id="LC2048" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L2049" class="blob-num js-line-number" data-line-number="2049"></td>
        <td id="LC2049" class="blob-code blob-code-inner js-file-line">					OperatingMode = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L2050" class="blob-num js-line-number" data-line-number="2050"></td>
        <td id="LC2050" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L2051" class="blob-num js-line-number" data-line-number="2051"></td>
        <td id="LC2051" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L2052" class="blob-num js-line-number" data-line-number="2052"></td>
        <td id="LC2052" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L2053" class="blob-num js-line-number" data-line-number="2053"></td>
        <td id="LC2053" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">if</span> (prev_prev_acceleration &lt; -<span class="pl-c1">1</span> &amp;&amp; prev_acceleration &lt; -<span class="pl-c1">1</span> &amp;&amp; acceleration &lt; -<span class="pl-c1">1</span>)</td>
      </tr>
      <tr>
        <td id="L2054" class="blob-num js-line-number" data-line-number="2054"></td>
        <td id="LC2054" class="blob-code blob-code-inner js-file-line">					{</td>
      </tr>
      <tr>
        <td id="L2055" class="blob-num js-line-number" data-line-number="2055"></td>
        <td id="LC2055" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">if</span> (prev_prev_acceleration != -<span class="pl-c1">99999</span>.<span class="pl-c1">0f</span> || prev_acceleration != -<span class="pl-c1">99999</span>.<span class="pl-c1">0f</span>)</td>
      </tr>
      <tr>
        <td id="L2056" class="blob-num js-line-number" data-line-number="2056"></td>
        <td id="LC2056" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L2057" class="blob-num js-line-number" data-line-number="2057"></td>
        <td id="LC2057" class="blob-code blob-code-inner js-file-line">							OperatingMode = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L2058" class="blob-num js-line-number" data-line-number="2058"></td>
        <td id="LC2058" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L2059" class="blob-num js-line-number" data-line-number="2059"></td>
        <td id="LC2059" class="blob-code blob-code-inner js-file-line">					}</td>
      </tr>
      <tr>
        <td id="L2060" class="blob-num js-line-number" data-line-number="2060"></td>
        <td id="LC2060" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L2061" class="blob-num js-line-number" data-line-number="2061"></td>
        <td id="LC2061" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2062" class="blob-num js-line-number" data-line-number="2062"></td>
        <td id="LC2062" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> ((SpeedBySecond / <span class="pl-c1">0</span>.<span class="pl-c1">44704f</span>) &gt;= -<span class="pl-c1">1</span>.<span class="pl-c1">0f</span> &amp;&amp; (SpeedBySecond / <span class="pl-c1">0</span>.<span class="pl-c1">44704f</span>) &lt; <span class="pl-c1">1</span>.<span class="pl-c1">0f</span>)</td>
      </tr>
      <tr>
        <td id="L2063" class="blob-num js-line-number" data-line-number="2063"></td>
        <td id="LC2063" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L2064" class="blob-num js-line-number" data-line-number="2064"></td>
        <td id="LC2064" class="blob-code blob-code-inner js-file-line">					OperatingMode = <span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L2065" class="blob-num js-line-number" data-line-number="2065"></td>
        <td id="LC2065" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L2066" class="blob-num js-line-number" data-line-number="2066"></td>
        <td id="LC2066" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2067" class="blob-num js-line-number" data-line-number="2067"></td>
        <td id="LC2067" class="blob-code blob-code-inner js-file-line">				pVehicle-&gt;m_OperatingModeCount[OperatingMode]++;</td>
      </tr>
      <tr>
        <td id="L2068" class="blob-num js-line-number" data-line-number="2068"></td>
        <td id="LC2068" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">int</span> integer_speed = SpeedBySecond / <span class="pl-c1">0</span>.<span class="pl-c1">44704f</span>;  <span class="pl-c">// convert meter per second to mile per hour</span></td>
      </tr>
      <tr>
        <td id="L2069" class="blob-num js-line-number" data-line-number="2069"></td>
        <td id="LC2069" class="blob-code blob-code-inner js-file-line">				pVehicle-&gt;m_SpeedCount[integer_speed]++;</td>
      </tr>
      <tr>
        <td id="L2070" class="blob-num js-line-number" data-line-number="2070"></td>
        <td id="LC2070" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2071" class="blob-num js-line-number" data-line-number="2071"></td>
        <td id="LC2071" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2072" class="blob-num js-line-number" data-line-number="2072"></td>
        <td id="LC2072" class="blob-code blob-code-inner js-file-line">				pVehicle-&gt;m_PrevSpeed  = SpeedBySecond;</td>
      </tr>
      <tr>
        <td id="L2073" class="blob-num js-line-number" data-line-number="2073"></td>
        <td id="LC2073" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2074" class="blob-num js-line-number" data-line-number="2074"></td>
        <td id="LC2074" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">int</span> vehicle_type = pVehicle-&gt;m_VehicleType;</td>
      </tr>
      <tr>
        <td id="L2075" class="blob-num js-line-number" data-line-number="2075"></td>
        <td id="LC2075" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2076" class="blob-num js-line-number" data-line-number="2076"></td>
        <td id="LC2076" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">int</span> age = pVehicle-&gt;m_Age ;</td>
      </tr>
      <tr>
        <td id="L2077" class="blob-num js-line-number" data-line-number="2077"></td>
        <td id="LC2077" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2078" class="blob-num js-line-number" data-line-number="2078"></td>
        <td id="LC2078" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">int</span> time_in_min = start_time / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND / <span class="pl-c1">60</span>;</td>
      </tr>
      <tr>
        <td id="L2079" class="blob-num js-line-number" data-line-number="2079"></td>
        <td id="LC2079" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2080" class="blob-num js-line-number" data-line-number="2080"></td>
        <td id="LC2080" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (EmissionRateData[vehicle_type][OperatingMode][age].<span class="pl-smi">bInitialized</span> == <span class="pl-c1">false</span>)</td>
      </tr>
      <tr>
        <td id="L2081" class="blob-num js-line-number" data-line-number="2081"></td>
        <td id="LC2081" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L2082" class="blob-num js-line-number" data-line-number="2082"></td>
        <td id="LC2082" class="blob-code blob-code-inner js-file-line">					<span class="pl-c">// try without data</span></td>
      </tr>
      <tr>
        <td id="L2083" class="blob-num js-line-number" data-line-number="2083"></td>
        <td id="LC2083" class="blob-code blob-code-inner js-file-line">					age = <span class="pl-c1">5</span> * <span class="pl-c1">int</span>(age*<span class="pl-c1">1.0</span> / <span class="pl-c1">5</span> + <span class="pl-c1">0.5</span>);</td>
      </tr>
      <tr>
        <td id="L2084" class="blob-num js-line-number" data-line-number="2084"></td>
        <td id="LC2084" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2085" class="blob-num js-line-number" data-line-number="2085"></td>
        <td id="LC2085" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L2086" class="blob-num js-line-number" data-line-number="2086"></td>
        <td id="LC2086" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2087" class="blob-num js-line-number" data-line-number="2087"></td>
        <td id="LC2087" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2088" class="blob-num js-line-number" data-line-number="2088"></td>
        <td id="LC2088" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span>(EmissionRateData[vehicle_type][OperatingMode][age].<span class="pl-smi">bInitialized</span> == <span class="pl-c1">false</span>)</td>
      </tr>
      <tr>
        <td id="L2089" class="blob-num js-line-number" data-line-number="2089"></td>
        <td id="LC2089" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L2090" class="blob-num js-line-number" data-line-number="2090"></td>
        <td id="LC2090" class="blob-code blob-code-inner js-file-line">					cout &lt;&lt; <span class="pl-s"><span class="pl-pds">&quot;</span>Emission rate data are not available for vehicle type = <span class="pl-pds">&quot;</span></span> &lt;&lt;  vehicle_type </td>
      </tr>
      <tr>
        <td id="L2091" class="blob-num js-line-number" data-line-number="2091"></td>
        <td id="LC2091" class="blob-code blob-code-inner js-file-line">						&lt;&lt; <span class="pl-s"><span class="pl-pds">&quot;</span>, operating mode = <span class="pl-pds">&quot;</span></span> &lt;&lt; OperatingMode </td>
      </tr>
      <tr>
        <td id="L2092" class="blob-num js-line-number" data-line-number="2092"></td>
        <td id="LC2092" class="blob-code blob-code-inner js-file-line">						&lt;&lt; <span class="pl-s"><span class="pl-pds">&quot;</span> and age = <span class="pl-pds">&quot;</span></span> &lt;&lt; age &lt;&lt; endl;</td>
      </tr>
      <tr>
        <td id="L2093" class="blob-num js-line-number" data-line-number="2093"></td>
        <td id="LC2093" class="blob-code blob-code-inner js-file-line">					<span class="pl-c1">g_ProgramStop</span>();</td>
      </tr>
      <tr>
        <td id="L2094" class="blob-num js-line-number" data-line-number="2094"></td>
        <td id="LC2094" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L2095" class="blob-num js-line-number" data-line-number="2095"></td>
        <td id="LC2095" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2096" class="blob-num js-line-number" data-line-number="2096"></td>
        <td id="LC2096" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">float</span> Energy = EmissionRateData[vehicle_type][OperatingMode][age].<span class="pl-smi">meanBaseRate_TotalEnergy</span> / <span class="pl-c1">3600</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L2097" class="blob-num js-line-number" data-line-number="2097"></td>
        <td id="LC2097" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">float</span> CO2 = EmissionRateData[vehicle_type][OperatingMode][age].<span class="pl-smi">meanBaseRate_CO2</span> / <span class="pl-c1">3600</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L2098" class="blob-num js-line-number" data-line-number="2098"></td>
        <td id="LC2098" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">float</span> NOX = EmissionRateData[vehicle_type][OperatingMode][age].<span class="pl-smi">meanBaseRate_NOX</span> / <span class="pl-c1">3600</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L2099" class="blob-num js-line-number" data-line-number="2099"></td>
        <td id="LC2099" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">float</span> CO = EmissionRateData[vehicle_type][OperatingMode][age].<span class="pl-smi">meanBaseRate_CO</span> / <span class="pl-c1">3600</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L2100" class="blob-num js-line-number" data-line-number="2100"></td>
        <td id="LC2100" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">float</span> HC = EmissionRateData[vehicle_type][OperatingMode][age].<span class="pl-smi">meanBaseRate_HC</span> / <span class="pl-c1">3600</span>.<span class="pl-c1">0f</span>;</td>
      </tr>
      <tr>
        <td id="L2101" class="blob-num js-line-number" data-line-number="2101"></td>
        <td id="LC2101" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2102" class="blob-num js-line-number" data-line-number="2102"></td>
        <td id="LC2102" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">if</span>(time_in_min ==<span class="pl-c1">430</span> &amp;&amp; m_FromNodeNumber == <span class="pl-c1">1</span> &amp;&amp; m_ToNodeNumber == <span class="pl-c1">3</span>)</td>
      </tr>
      <tr>
        <td id="L2103" class="blob-num js-line-number" data-line-number="2103"></td>
        <td id="LC2103" class="blob-code blob-code-inner js-file-line">					{</td>
      </tr>
      <tr>
        <td id="L2104" class="blob-num js-line-number" data-line-number="2104"></td>
        <td id="LC2104" class="blob-code blob-code-inner js-file-line">					<span class="pl-c1">TRACE</span>(<span class="pl-s"><span class="pl-pds">&quot;</span><span class="pl-pds">&quot;</span></span>);</td>
      </tr>
      <tr>
        <td id="L2105" class="blob-num js-line-number" data-line-number="2105"></td>
        <td id="LC2105" class="blob-code blob-code-inner js-file-line">					</td>
      </tr>
      <tr>
        <td id="L2106" class="blob-num js-line-number" data-line-number="2106"></td>
        <td id="LC2106" class="blob-code blob-code-inner js-file-line">					}</td>
      </tr>
      <tr>
        <td id="L2107" class="blob-num js-line-number" data-line-number="2107"></td>
        <td id="LC2107" class="blob-code blob-code-inner js-file-line">				m_VehicleDataVector[LaneNo].<span class="pl-smi">m_LaneEmissionVector</span> [time_in_min].<span class="pl-smi">Energy</span> += Energy;</td>
      </tr>
      <tr>
        <td id="L2108" class="blob-num js-line-number" data-line-number="2108"></td>
        <td id="LC2108" class="blob-code blob-code-inner js-file-line">				m_VehicleDataVector[LaneNo].<span class="pl-smi">m_LaneEmissionVector</span> [time_in_min].<span class="pl-smi">CO2</span> += CO2;</td>
      </tr>
      <tr>
        <td id="L2109" class="blob-num js-line-number" data-line-number="2109"></td>
        <td id="LC2109" class="blob-code blob-code-inner js-file-line">				m_VehicleDataVector[LaneNo].<span class="pl-smi">m_LaneEmissionVector</span> [time_in_min].<span class="pl-smi">NOX</span> += NOX;</td>
      </tr>
      <tr>
        <td id="L2110" class="blob-num js-line-number" data-line-number="2110"></td>
        <td id="LC2110" class="blob-code blob-code-inner js-file-line">				m_VehicleDataVector[LaneNo].<span class="pl-smi">m_LaneEmissionVector</span> [time_in_min].<span class="pl-smi">CO</span> += CO;</td>
      </tr>
      <tr>
        <td id="L2111" class="blob-num js-line-number" data-line-number="2111"></td>
        <td id="LC2111" class="blob-code blob-code-inner js-file-line">				m_VehicleDataVector[LaneNo].<span class="pl-smi">m_LaneEmissionVector</span> [time_in_min].<span class="pl-smi">HC</span> += HC;</td>
      </tr>
      <tr>
        <td id="L2112" class="blob-num js-line-number" data-line-number="2112"></td>
        <td id="LC2112" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2113" class="blob-num js-line-number" data-line-number="2113"></td>
        <td id="LC2113" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (pVehicle-&gt;m_AgentID  ==  g_TargetVehicleID_OutputSecondBySecondEmissionData || </td>
      </tr>
      <tr>
        <td id="L2114" class="blob-num js-line-number" data-line-number="2114"></td>
        <td id="LC2114" class="blob-code blob-code-inner js-file-line">					(g_OutputSecondBySecondEmissionData &amp;&amp; pVehicle-&gt;m_DepartureTime &gt;= g_start_departure_time_for_output_second_by_second_emission_data </td>
      </tr>
      <tr>
        <td id="L2115" class="blob-num js-line-number" data-line-number="2115"></td>
        <td id="LC2115" class="blob-code blob-code-inner js-file-line">					&amp;&amp; pVehicle-&gt;m_DepartureTime &lt;= g_end_departure_time_for_output_second_by_second_emission_data</td>
      </tr>
      <tr>
        <td id="L2116" class="blob-num js-line-number" data-line-number="2116"></td>
        <td id="LC2116" class="blob-code blob-code-inner js-file-line">					&amp;&amp; pVehicle-&gt;m_bDetailedEmissionOutput))  <span class="pl-c">// record data</span></td>
      </tr>
      <tr>
        <td id="L2117" class="blob-num js-line-number" data-line-number="2117"></td>
        <td id="LC2117" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L2118" class="blob-num js-line-number" data-line-number="2118"></td>
        <td id="LC2118" class="blob-code blob-code-inner js-file-line">					VehicleSpeedProfileData element;</td>
      </tr>
      <tr>
        <td id="L2119" class="blob-num js-line-number" data-line-number="2119"></td>
        <td id="LC2119" class="blob-code blob-code-inner js-file-line">					element.<span class="pl-smi">FromNodeNumber</span> = <span class="pl-v">this</span>-&gt;m_FromNodeNumber ;</td>
      </tr>
      <tr>
        <td id="L2120" class="blob-num js-line-number" data-line-number="2120"></td>
        <td id="LC2120" class="blob-code blob-code-inner js-file-line">					element.<span class="pl-smi">ToNodeNumber</span>  = <span class="pl-v">this</span>-&gt;m_ToNodeNumber ;</td>
      </tr>
      <tr>
        <td id="L2121" class="blob-num js-line-number" data-line-number="2121"></td>
        <td id="LC2121" class="blob-code blob-code-inner js-file-line">					element.<span class="pl-smi">TimeOnThisLinkInSecond</span> = (t - start_time) / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;</td>
      </tr>
      <tr>
        <td id="L2122" class="blob-num js-line-number" data-line-number="2122"></td>
        <td id="LC2122" class="blob-code blob-code-inner js-file-line">					element.<span class="pl-smi">Speed</span> = SpeedBySecond / <span class="pl-c1">0</span>.<span class="pl-c1">44704f</span>; <span class="pl-c">// km/h to mph</span></td>
      </tr>
      <tr>
        <td id="L2123" class="blob-num js-line-number" data-line-number="2123"></td>
        <td id="LC2123" class="blob-code blob-code-inner js-file-line">					element.<span class="pl-smi">Acceleration</span> = acceleration * <span class="pl-c1">3.28084</span>; <span class="pl-c">//meter/sec^2 to feet/sec^2</span></td>
      </tr>
      <tr>
        <td id="L2124" class="blob-num js-line-number" data-line-number="2124"></td>
        <td id="LC2124" class="blob-code blob-code-inner js-file-line">					element.<span class="pl-smi">OperationMode</span> = OperatingMode;</td>
      </tr>
      <tr>
        <td id="L2125" class="blob-num js-line-number" data-line-number="2125"></td>
        <td id="LC2125" class="blob-code blob-code-inner js-file-line">					element.<span class="pl-smi">VSP</span>  = vsp;</td>
      </tr>
      <tr>
        <td id="L2126" class="blob-num js-line-number" data-line-number="2126"></td>
        <td id="LC2126" class="blob-code blob-code-inner js-file-line">					element.<span class="pl-smi">SpeedBinNo</span> = <span class="pl-c1">GetSpeedBinNo</span>(element.<span class="pl-smi">Speed</span>);</td>
      </tr>
      <tr>
        <td id="L2127" class="blob-num js-line-number" data-line-number="2127"></td>
        <td id="LC2127" class="blob-code blob-code-inner js-file-line">					element.<span class="pl-smi">VSPBinNo</span> =  <span class="pl-c1">GetVSPBinNo</span>(vsp, element.<span class="pl-smi">Speed</span>);</td>
      </tr>
      <tr>
        <td id="L2128" class="blob-num js-line-number" data-line-number="2128"></td>
        <td id="LC2128" class="blob-code blob-code-inner js-file-line">					element.<span class="pl-smi">Energy</span> = Energy;</td>
      </tr>
      <tr>
        <td id="L2129" class="blob-num js-line-number" data-line-number="2129"></td>
        <td id="LC2129" class="blob-code blob-code-inner js-file-line">					element.<span class="pl-smi">CO2</span> = CO2;</td>
      </tr>
      <tr>
        <td id="L2130" class="blob-num js-line-number" data-line-number="2130"></td>
        <td id="LC2130" class="blob-code blob-code-inner js-file-line">					element.<span class="pl-smi">NOX</span> = NOX;</td>
      </tr>
      <tr>
        <td id="L2131" class="blob-num js-line-number" data-line-number="2131"></td>
        <td id="LC2131" class="blob-code blob-code-inner js-file-line">					element.<span class="pl-smi">CO</span> = CO;</td>
      </tr>
      <tr>
        <td id="L2132" class="blob-num js-line-number" data-line-number="2132"></td>
        <td id="LC2132" class="blob-code blob-code-inner js-file-line">					element.<span class="pl-smi">HC</span> = HC;</td>
      </tr>
      <tr>
        <td id="L2133" class="blob-num js-line-number" data-line-number="2133"></td>
        <td id="LC2133" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2134" class="blob-num js-line-number" data-line-number="2134"></td>
        <td id="LC2134" class="blob-code blob-code-inner js-file-line">					pVehicle-&gt;m_SpeedProfile[t] = element;</td>
      </tr>
      <tr>
        <td id="L2135" class="blob-num js-line-number" data-line-number="2135"></td>
        <td id="LC2135" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L2136" class="blob-num js-line-number" data-line-number="2136"></td>
        <td id="LC2136" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2137" class="blob-num js-line-number" data-line-number="2137"></td>
        <td id="LC2137" class="blob-code blob-code-inner js-file-line">				prev_prev_acceleration = prev_acceleration;</td>
      </tr>
      <tr>
        <td id="L2138" class="blob-num js-line-number" data-line-number="2138"></td>
        <td id="LC2138" class="blob-code blob-code-inner js-file-line">				prev_acceleration = acceleration * <span class="pl-c1">0</span>.<span class="pl-c1">44704f</span>;				</td>
      </tr>
      <tr>
        <td id="L2139" class="blob-num js-line-number" data-line-number="2139"></td>
        <td id="LC2139" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L2140" class="blob-num js-line-number" data-line-number="2140"></td>
        <td id="LC2140" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2141" class="blob-num js-line-number" data-line-number="2141"></td>
        <td id="LC2141" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">float</span> *tmp = pLeaderPositions;</td>
      </tr>
      <tr>
        <td id="L2142" class="blob-num js-line-number" data-line-number="2142"></td>
        <td id="LC2142" class="blob-code blob-code-inner js-file-line">			pLeaderPositions = pFollowerPositions;</td>
      </tr>
      <tr>
        <td id="L2143" class="blob-num js-line-number" data-line-number="2143"></td>
        <td id="LC2143" class="blob-code blob-code-inner js-file-line">			pFollowerPositions = tmp;</td>
      </tr>
      <tr>
        <td id="L2144" class="blob-num js-line-number" data-line-number="2144"></td>
        <td id="LC2144" class="blob-code blob-code-inner js-file-line">			leaderInTime = start_time;</td>
      </tr>
      <tr>
        <td id="L2145" class="blob-num js-line-number" data-line-number="2145"></td>
        <td id="LC2145" class="blob-code blob-code-inner js-file-line">			leaderOutTime = end_time;</td>
      </tr>
      <tr>
        <td id="L2146" class="blob-num js-line-number" data-line-number="2146"></td>
        <td id="LC2146" class="blob-code blob-code-inner js-file-line">		} <span class="pl-c">// for each vehicle</span></td>
      </tr>
      <tr>
        <td id="L2147" class="blob-num js-line-number" data-line-number="2147"></td>
        <td id="LC2147" class="blob-code blob-code-inner js-file-line">	} <span class="pl-c">//for each lane</span></td>
      </tr>
      <tr>
        <td id="L2148" class="blob-num js-line-number" data-line-number="2148"></td>
        <td id="LC2148" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2149" class="blob-num js-line-number" data-line-number="2149"></td>
        <td id="LC2149" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//if (output != NULL)</span></td>
      </tr>
      <tr>
        <td id="L2150" class="blob-num js-line-number" data-line-number="2150"></td>
        <td id="LC2150" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//{</span></td>
      </tr>
      <tr>
        <td id="L2151" class="blob-num js-line-number" data-line-number="2151"></td>
        <td id="LC2151" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//	fclose(output);</span></td>
      </tr>
      <tr>
        <td id="L2152" class="blob-num js-line-number" data-line-number="2152"></td>
        <td id="LC2152" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//}</span></td>
      </tr>
      <tr>
        <td id="L2153" class="blob-num js-line-number" data-line-number="2153"></td>
        <td id="LC2153" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2154" class="blob-num js-line-number" data-line-number="2154"></td>
        <td id="LC2154" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//delete [] LeaderPositions;</span></td>
      </tr>
      <tr>
        <td id="L2155" class="blob-num js-line-number" data-line-number="2155"></td>
        <td id="LC2155" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//delete [] FollowerPositions;</span></td>
      </tr>
      <tr>
        <td id="L2156" class="blob-num js-line-number" data-line-number="2156"></td>
        <td id="LC2156" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">endif</span> </td>
      </tr>
      <tr>
        <td id="L2157" class="blob-num js-line-number" data-line-number="2157"></td>
        <td id="LC2157" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L2158" class="blob-num js-line-number" data-line-number="2158"></td>
        <td id="LC2158" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2159" class="blob-num js-line-number" data-line-number="2159"></td>
        <td id="LC2159" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2160" class="blob-num js-line-number" data-line-number="2160"></td>
        <td id="LC2160" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2161" class="blob-num js-line-number" data-line-number="2161"></td>
        <td id="LC2161" class="blob-code blob-code-inner js-file-line"><span class="pl-k">void</span> <span class="pl-en">DTALink::ComputeVSP_FastMethod</span>()  <span class="pl-c">// VSP: vehicle specific power</span></td>
      </tr>
      <tr>
        <td id="L2162" class="blob-num js-line-number" data-line-number="2162"></td>
        <td id="LC2162" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L2163" class="blob-num js-line-number" data-line-number="2163"></td>
        <td id="LC2163" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">ifdef</span> _large_memory_usage_emission</td>
      </tr>
      <tr>
        <td id="L2164" class="blob-num js-line-number" data-line-number="2164"></td>
        <td id="LC2164" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">// step 1: car following simulation</span></td>
      </tr>
      <tr>
        <td id="L2165" class="blob-num js-line-number" data-line-number="2165"></td>
        <td id="LC2165" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> link_length_in_meter = m_Length * <span class="pl-c1">1609</span>.<span class="pl-c1">344f</span>; <span class="pl-c">//1609.344f: mile to meters</span></td>
      </tr>
      <tr>
        <td id="L2166" class="blob-num js-line-number" data-line-number="2166"></td>
        <td id="LC2166" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2167" class="blob-num js-line-number" data-line-number="2167"></td>
        <td id="LC2167" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">float</span> BackwardWaveSpeed_in_meter_per_second = m_BackwardWaveSpeed  * <span class="pl-c1">1609</span>.<span class="pl-c1">344f</span> / <span class="pl-c1">3600</span>;</td>
      </tr>
      <tr>
        <td id="L2168" class="blob-num js-line-number" data-line-number="2168"></td>
        <td id="LC2168" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">for</span> (<span class="pl-k">int</span> LaneNo = <span class="pl-c1">0</span>; LaneNo &lt; m_OutflowNumLanes; LaneNo++)</td>
      </tr>
      <tr>
        <td id="L2169" class="blob-num js-line-number" data-line-number="2169"></td>
        <td id="LC2169" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L2170" class="blob-num js-line-number" data-line-number="2170"></td>
        <td id="LC2170" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">int</span> v;</td>
      </tr>
      <tr>
        <td id="L2171" class="blob-num js-line-number" data-line-number="2171"></td>
        <td id="LC2171" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">for</span> (v = <span class="pl-c1">0</span>; v&lt;m_VehicleDataVector[LaneNo].<span class="pl-smi">LaneData</span>.<span class="pl-c1">size</span>(); v++)</td>
      </tr>
      <tr>
        <td id="L2172" class="blob-num js-line-number" data-line-number="2172"></td>
        <td id="LC2172" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L2173" class="blob-num js-line-number" data-line-number="2173"></td>
        <td id="LC2173" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">float</span> distance = <span class="pl-c1">0</span>;  <span class="pl-c">// from the starting point of the link to the current location</span></td>
      </tr>
      <tr>
        <td id="L2174" class="blob-num js-line-number" data-line-number="2174"></td>
        <td id="LC2174" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">float</span> prev_distance = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L2175" class="blob-num js-line-number" data-line-number="2175"></td>
        <td id="LC2175" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2176" class="blob-num js-line-number" data-line-number="2176"></td>
        <td id="LC2176" class="blob-code blob-code-inner js-file-line">			DTAVehicle* pVehicle = g_VehicleMap[m_VehicleDataVector[LaneNo].<span class="pl-smi">LaneData</span>[v].<span class="pl-smi">VehicleID</span>];</td>
      </tr>
      <tr>
        <td id="L2177" class="blob-num js-line-number" data-line-number="2177"></td>
        <td id="LC2177" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2178" class="blob-num js-line-number" data-line-number="2178"></td>
        <td id="LC2178" class="blob-code blob-code-inner js-file-line">			std::map&lt;<span class="pl-k">int</span>, <span class="pl-k">int</span>&gt; l_OperatingModeCount;</td>
      </tr>
      <tr>
        <td id="L2179" class="blob-num js-line-number" data-line-number="2179"></td>
        <td id="LC2179" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2180" class="blob-num js-line-number" data-line-number="2180"></td>
        <td id="LC2180" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">int</span> SequentialLinkNo = m_VehicleDataVector[LaneNo].<span class="pl-smi">LaneData</span>[v].<span class="pl-smi">SequentialLinkNo</span>;</td>
      </tr>
      <tr>
        <td id="L2181" class="blob-num js-line-number" data-line-number="2181"></td>
        <td id="LC2181" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2182" class="blob-num js-line-number" data-line-number="2182"></td>
        <td id="LC2182" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">for</span> (<span class="pl-k">int</span> t = m_VehicleDataVector[LaneNo].<span class="pl-smi">LaneData</span>[v].<span class="pl-smi">StartTime_in_SimulationInterval</span>;</td>
      </tr>
      <tr>
        <td id="L2183" class="blob-num js-line-number" data-line-number="2183"></td>
        <td id="LC2183" class="blob-code blob-code-inner js-file-line">				t &lt; m_VehicleDataVector[LaneNo].<span class="pl-smi">LaneData</span>[v].<span class="pl-smi">EndTime_in_SimulationInterval</span>; t += <span class="pl-c1">1</span> * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND)</td>
      </tr>
      <tr>
        <td id="L2184" class="blob-num js-line-number" data-line-number="2184"></td>
        <td id="LC2184" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L2185" class="blob-num js-line-number" data-line-number="2185"></td>
        <td id="LC2185" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">//calculate free-flow position</span></td>
      </tr>
      <tr>
        <td id="L2186" class="blob-num js-line-number" data-line-number="2186"></td>
        <td id="LC2186" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">//xiF(t) = xi(t-τ) + vf(τ)</span></td>
      </tr>
      <tr>
        <td id="L2187" class="blob-num js-line-number" data-line-number="2187"></td>
        <td id="LC2187" class="blob-code blob-code-inner js-file-line">				prev_distance = distance;</td>
      </tr>
      <tr>
        <td id="L2188" class="blob-num js-line-number" data-line-number="2188"></td>
        <td id="LC2188" class="blob-code blob-code-inner js-file-line">				distance = <span class="pl-c1">min</span>(link_length_in_meter, distance + m_VehicleDataVector[LaneNo].<span class="pl-smi">LaneData</span>[v].<span class="pl-smi">FreeflowDistance_per_SimulationInterval</span>);</td>
      </tr>
      <tr>
        <td id="L2189" class="blob-num js-line-number" data-line-number="2189"></td>
        <td id="LC2189" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">//					TRACE(&quot;veh %d, time%d,%f\n&quot;,v,t,VechileDistanceAry[v][t]);</span></td>
      </tr>
      <tr>
        <td id="L2190" class="blob-num js-line-number" data-line-number="2190"></td>
        <td id="LC2190" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2191" class="blob-num js-line-number" data-line-number="2191"></td>
        <td id="LC2191" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">//calculate congested position</span></td>
      </tr>
      <tr>
        <td id="L2192" class="blob-num js-line-number" data-line-number="2192"></td>
        <td id="LC2192" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">float</span> BWTT_in_second = (link_length_in_meter - distance) / <span class="pl-c1">max</span>(<span class="pl-c1">0.001</span>, BackwardWaveSpeed_in_meter_per_second);</td>
      </tr>
      <tr>
        <td id="L2193" class="blob-num js-line-number" data-line-number="2193"></td>
        <td id="LC2193" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">int</span> time_t_minus_BWTT = t - BWTT_in_second*NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND; <span class="pl-c">// need to convert time in second to time in simulation time interval</span></td>
      </tr>
      <tr>
        <td id="L2194" class="blob-num js-line-number" data-line-number="2194"></td>
        <td id="LC2194" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">int</span> DTASimulationTimeIntervalNo = time_t_minus_BWTT*g_DTASimulationInterval;</td>
      </tr>
      <tr>
        <td id="L2195" class="blob-num js-line-number" data-line-number="2195"></td>
        <td id="LC2195" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2196" class="blob-num js-line-number" data-line-number="2196"></td>
        <td id="LC2196" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (DTASimulationTimeIntervalNo &lt; <span class="pl-c1">0</span>)</td>
      </tr>
      <tr>
        <td id="L2197" class="blob-num js-line-number" data-line-number="2197"></td>
        <td id="LC2197" class="blob-code blob-code-inner js-file-line">					DTASimulationTimeIntervalNo = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L2198" class="blob-num js-line-number" data-line-number="2198"></td>
        <td id="LC2198" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (DTASimulationTimeIntervalNo &gt;= m_CumuDeparturelFlow.<span class="pl-c1">size</span>() - <span class="pl-c1">1</span>)</td>
      </tr>
      <tr>
        <td id="L2199" class="blob-num js-line-number" data-line-number="2199"></td>
        <td id="LC2199" class="blob-code blob-code-inner js-file-line">					DTASimulationTimeIntervalNo = m_CumuDeparturelFlow.<span class="pl-c1">size</span>() - <span class="pl-c1">2</span>;</td>
      </tr>
      <tr>
        <td id="L2200" class="blob-num js-line-number" data-line-number="2200"></td>
        <td id="LC2200" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2201" class="blob-num js-line-number" data-line-number="2201"></td>
        <td id="LC2201" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">int</span> DownstreamCumulativeDepartureCount = m_CumuDeparturelFlow[DTASimulationTimeIntervalNo];</td>
      </tr>
      <tr>
        <td id="L2202" class="blob-num js-line-number" data-line-number="2202"></td>
        <td id="LC2202" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">// reference: Newell&#39;s 3-detector theory </span></td>
      </tr>
      <tr>
        <td id="L2203" class="blob-num js-line-number" data-line-number="2203"></td>
        <td id="LC2203" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2204" class="blob-num js-line-number" data-line-number="2204"></td>
        <td id="LC2204" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">float</span> SpeedBySecond = (distance - prev_distance)* <span class="pl-c1">0</span>.<span class="pl-c1">44704f</span>; <span class="pl-c">// // 1 mph = 0.44704 meters per second</span></td>
      </tr>
      <tr>
        <td id="L2205" class="blob-num js-line-number" data-line-number="2205"></td>
        <td id="LC2205" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2206" class="blob-num js-line-number" data-line-number="2206"></td>
        <td id="LC2206" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">// output speed per second</span></td>
      </tr>
      <tr>
        <td id="L2207" class="blob-num js-line-number" data-line-number="2207"></td>
        <td id="LC2207" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2208" class="blob-num js-line-number" data-line-number="2208"></td>
        <td id="LC2208" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (t%NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND == <span class="pl-c1">0</span>)  <span class="pl-c">// per_second</span></td>
      </tr>
      <tr>
        <td id="L2209" class="blob-num js-line-number" data-line-number="2209"></td>
        <td id="LC2209" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L2210" class="blob-num js-line-number" data-line-number="2210"></td>
        <td id="LC2210" class="blob-code blob-code-inner js-file-line">					<span class="pl-c">// for active vehicles (with positive speed or positive distance</span></td>
      </tr>
      <tr>
        <td id="L2211" class="blob-num js-line-number" data-line-number="2211"></td>
        <td id="LC2211" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2212" class="blob-num js-line-number" data-line-number="2212"></td>
        <td id="LC2212" class="blob-code blob-code-inner js-file-line">					<span class="pl-c">// different lanes have different vehicle numbers, so we should not have openMP conflicts here</span></td>
      </tr>
      <tr>
        <td id="L2213" class="blob-num js-line-number" data-line-number="2213"></td>
        <td id="LC2213" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">float</span> accelation = SpeedBySecond - pVehicle-&gt;m_PrevSpeed;</td>
      </tr>
      <tr>
        <td id="L2214" class="blob-num js-line-number" data-line-number="2214"></td>
        <td id="LC2214" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2215" class="blob-num js-line-number" data-line-number="2215"></td>
        <td id="LC2215" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">int</span> OperatingMode = <span class="pl-c1">ComputeOperatingModeFromSpeed</span>(SpeedBySecond, accelation);</td>
      </tr>
      <tr>
        <td id="L2216" class="blob-num js-line-number" data-line-number="2216"></td>
        <td id="LC2216" class="blob-code blob-code-inner js-file-line">					pVehicle-&gt;m_OperatingModeCount[OperatingMode] += <span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L2217" class="blob-num js-line-number" data-line-number="2217"></td>
        <td id="LC2217" class="blob-code blob-code-inner js-file-line">					pVehicle-&gt;m_PrevSpeed = SpeedBySecond;</td>
      </tr>
      <tr>
        <td id="L2218" class="blob-num js-line-number" data-line-number="2218"></td>
        <td id="LC2218" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2219" class="blob-num js-line-number" data-line-number="2219"></td>
        <td id="LC2219" class="blob-code blob-code-inner js-file-line">					l_OperatingModeCount[OperatingMode] += <span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L2220" class="blob-num js-line-number" data-line-number="2220"></td>
        <td id="LC2220" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2221" class="blob-num js-line-number" data-line-number="2221"></td>
        <td id="LC2221" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L2222" class="blob-num js-line-number" data-line-number="2222"></td>
        <td id="LC2222" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2223" class="blob-num js-line-number" data-line-number="2223"></td>
        <td id="LC2223" class="blob-code blob-code-inner js-file-line">			}  <span class="pl-c">// for each time t</span></td>
      </tr>
      <tr>
        <td id="L2224" class="blob-num js-line-number" data-line-number="2224"></td>
        <td id="LC2224" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2225" class="blob-num js-line-number" data-line-number="2225"></td>
        <td id="LC2225" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">// calculate subtotal emissions for each vehicle</span></td>
      </tr>
      <tr>
        <td id="L2226" class="blob-num js-line-number" data-line-number="2226"></td>
        <td id="LC2226" class="blob-code blob-code-inner js-file-line">			CVehicleEmission <span class="pl-smi">element</span>(pVehicle-&gt;m_VehicleType, pVehicle-&gt;m_OperatingModeCount, pVehicle-&gt;m_Age);</td>
      </tr>
      <tr>
        <td id="L2227" class="blob-num js-line-number" data-line-number="2227"></td>
        <td id="LC2227" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2228" class="blob-num js-line-number" data-line-number="2228"></td>
        <td id="LC2228" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">// add the vehicle specific emissions to link-based emission statistics</span></td>
      </tr>
      <tr>
        <td id="L2229" class="blob-num js-line-number" data-line-number="2229"></td>
        <td id="LC2229" class="blob-code blob-code-inner js-file-line">			m_TotalEnergy += element.<span class="pl-smi">Energy</span>;</td>
      </tr>
      <tr>
        <td id="L2230" class="blob-num js-line-number" data-line-number="2230"></td>
        <td id="LC2230" class="blob-code blob-code-inner js-file-line">			m_CO2 += element.<span class="pl-smi">CO2</span>;</td>
      </tr>
      <tr>
        <td id="L2231" class="blob-num js-line-number" data-line-number="2231"></td>
        <td id="LC2231" class="blob-code blob-code-inner js-file-line">			m_NOX += element.<span class="pl-smi">NOX</span>;</td>
      </tr>
      <tr>
        <td id="L2232" class="blob-num js-line-number" data-line-number="2232"></td>
        <td id="LC2232" class="blob-code blob-code-inner js-file-line">			m_CO += element.<span class="pl-smi">CO</span>;</td>
      </tr>
      <tr>
        <td id="L2233" class="blob-num js-line-number" data-line-number="2233"></td>
        <td id="LC2233" class="blob-code blob-code-inner js-file-line">			m_HC += element.<span class="pl-smi">HC</span>;</td>
      </tr>
      <tr>
        <td id="L2234" class="blob-num js-line-number" data-line-number="2234"></td>
        <td id="LC2234" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2235" class="blob-num js-line-number" data-line-number="2235"></td>
        <td id="LC2235" class="blob-code blob-code-inner js-file-line">		}  <span class="pl-c">// for active vehicle</span></td>
      </tr>
      <tr>
        <td id="L2236" class="blob-num js-line-number" data-line-number="2236"></td>
        <td id="LC2236" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2237" class="blob-num js-line-number" data-line-number="2237"></td>
        <td id="LC2237" class="blob-code blob-code-inner js-file-line">	} <span class="pl-c">// for each lane</span></td>
      </tr>
      <tr>
        <td id="L2238" class="blob-num js-line-number" data-line-number="2238"></td>
        <td id="LC2238" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2239" class="blob-num js-line-number" data-line-number="2239"></td>
        <td id="LC2239" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">endif</span></td>
      </tr>
      <tr>
        <td id="L2240" class="blob-num js-line-number" data-line-number="2240"></td>
        <td id="LC2240" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L2241" class="blob-num js-line-number" data-line-number="2241"></td>
        <td id="LC2241" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2242" class="blob-num js-line-number" data-line-number="2242"></td>
        <td id="LC2242" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2243" class="blob-num js-line-number" data-line-number="2243"></td>
        <td id="LC2243" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2244" class="blob-num js-line-number" data-line-number="2244"></td>
        <td id="LC2244" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2245" class="blob-num js-line-number" data-line-number="2245"></td>
        <td id="LC2245" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2246" class="blob-num js-line-number" data-line-number="2246"></td>
        <td id="LC2246" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2247" class="blob-num js-line-number" data-line-number="2247"></td>
        <td id="LC2247" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2248" class="blob-num js-line-number" data-line-number="2248"></td>
        <td id="LC2248" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2249" class="blob-num js-line-number" data-line-number="2249"></td>
        <td id="LC2249" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2250" class="blob-num js-line-number" data-line-number="2250"></td>
        <td id="LC2250" class="blob-code blob-code-inner js-file-line"><span class="pl-k">bool</span> <span class="pl-en">g_VehicularSimulation_version_2</span>(<span class="pl-k">int</span> DayNo, <span class="pl-k">double</span> CurrentTime, <span class="pl-k">int</span> meso_simulation_time_interval_no, <span class="pl-k">int</span> TrafficFlowModelFlag)</td>
      </tr>
      <tr>
        <td id="L2251" class="blob-num js-line-number" data-line-number="2251"></td>
        <td id="LC2251" class="blob-code blob-code-inner js-file-line">{</td>
      </tr>
      <tr>
        <td id="L2252" class="blob-num js-line-number" data-line-number="2252"></td>
        <td id="LC2252" class="blob-code blob-code-inner js-file-line">	std::set&lt;DTANode*&gt;::iterator iterNode;</td>
      </tr>
      <tr>
        <td id="L2253" class="blob-num js-line-number" data-line-number="2253"></td>
        <td id="LC2253" class="blob-code blob-code-inner js-file-line">	std::set&lt;DTALink*&gt;::iterator iterLink;</td>
      </tr>
      <tr>
        <td id="L2254" class="blob-num js-line-number" data-line-number="2254"></td>
        <td id="LC2254" class="blob-code blob-code-inner js-file-line">	std::map&lt;<span class="pl-k">int</span>, DTAVehicle*&gt;::iterator iterVM;</td>
      </tr>
      <tr>
        <td id="L2255" class="blob-num js-line-number" data-line-number="2255"></td>
        <td id="LC2255" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span>   PathNodeList[MAX_NODE_SIZE_IN_A_PATH] = { <span class="pl-c1">0</span> };</td>
      </tr>
      <tr>
        <td id="L2256" class="blob-num js-line-number" data-line-number="2256"></td>
        <td id="LC2256" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2257" class="blob-num js-line-number" data-line-number="2257"></td>
        <td id="LC2257" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> time_stamp_in_min = <span class="pl-c1">int</span>(CurrentTime + <span class="pl-c1">0.0001</span>);</td>
      </tr>
      <tr>
        <td id="L2258" class="blob-num js-line-number" data-line-number="2258"></td>
        <td id="LC2258" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2259" class="blob-num js-line-number" data-line-number="2259"></td>
        <td id="LC2259" class="blob-code blob-code-inner js-file-line">	std::list&lt;struc_vehicle_item&gt;::iterator vii;</td>
      </tr>
      <tr>
        <td id="L2260" class="blob-num js-line-number" data-line-number="2260"></td>
        <td id="LC2260" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2261" class="blob-num js-line-number" data-line-number="2261"></td>
        <td id="LC2261" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> vehicle_id_trace = <span class="pl-c1">8297</span>;</td>
      </tr>
      <tr>
        <td id="L2262" class="blob-num js-line-number" data-line-number="2262"></td>
        <td id="LC2262" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> link_id_trace = -<span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L2263" class="blob-num js-line-number" data-line-number="2263"></td>
        <td id="LC2263" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">bool</span> debug_flag = <span class="pl-c1">true</span>;</td>
      </tr>
      <tr>
        <td id="L2264" class="blob-num js-line-number" data-line-number="2264"></td>
        <td id="LC2264" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2265" class="blob-num js-line-number" data-line-number="2265"></td>
        <td id="LC2265" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2266" class="blob-num js-line-number" data-line-number="2266"></td>
        <td id="LC2266" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//DTALite:</span></td>
      </tr>
      <tr>
        <td id="L2267" class="blob-num js-line-number" data-line-number="2267"></td>
        <td id="LC2267" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">// vertical queue data structure</span></td>
      </tr>
      <tr>
        <td id="L2268" class="blob-num js-line-number" data-line-number="2268"></td>
        <td id="LC2268" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">// each link  ExitQueue, EntranceQueue: (VehicleID, ReadyTime)</span></td>
      </tr>
      <tr>
        <td id="L2269" class="blob-num js-line-number" data-line-number="2269"></td>
        <td id="LC2269" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2270" class="blob-num js-line-number" data-line-number="2270"></td>
        <td id="LC2270" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">// step 0: /// update VMS path provision</span></td>
      </tr>
      <tr>
        <td id="L2271" class="blob-num js-line-number" data-line-number="2271"></td>
        <td id="LC2271" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2272" class="blob-num js-line-number" data-line-number="2272"></td>
        <td id="LC2272" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (meso_simulation_time_interval_no % (g_information_updating_interval_of_VMS_in_min * <span class="pl-c1">10</span>) == <span class="pl-c1">0</span>)  <span class="pl-c">//regenerate shortest path tree for VMS every g_information_updating_interval_of_VMS_in_min </span></td>
      </tr>
      <tr>
        <td id="L2273" class="blob-num js-line-number" data-line-number="2273"></td>
        <td id="LC2273" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L2274" class="blob-num js-line-number" data-line-number="2274"></td>
        <td id="LC2274" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">for</span> (<span class="pl-k">unsigned</span> li = <span class="pl-c1">0</span>; li&lt; g_LinkVector.<span class="pl-c1">size</span>(); li++)</td>
      </tr>
      <tr>
        <td id="L2275" class="blob-num js-line-number" data-line-number="2275"></td>
        <td id="LC2275" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L2276" class="blob-num js-line-number" data-line-number="2276"></td>
        <td id="LC2276" class="blob-code blob-code-inner js-file-line">			DTALink * pLink = g_LinkVector[li];</td>
      </tr>
      <tr>
        <td id="L2277" class="blob-num js-line-number" data-line-number="2277"></td>
        <td id="LC2277" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">for</span> (<span class="pl-k">unsigned</span> <span class="pl-k">int</span> m = <span class="pl-c1">0</span>; m&lt;pLink-&gt;MessageSignVector.<span class="pl-c1">size</span>(); m++)</td>
      </tr>
      <tr>
        <td id="L2278" class="blob-num js-line-number" data-line-number="2278"></td>
        <td id="LC2278" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L2279" class="blob-num js-line-number" data-line-number="2279"></td>
        <td id="LC2279" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2280" class="blob-num js-line-number" data-line-number="2280"></td>
        <td id="LC2280" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (g_LinkVector[li]-&gt;MessageSignVector[m].<span class="pl-smi">StartDayNo</span> &lt;= DayNo &amp;&amp;</td>
      </tr>
      <tr>
        <td id="L2281" class="blob-num js-line-number" data-line-number="2281"></td>
        <td id="LC2281" class="blob-code blob-code-inner js-file-line">					DayNo &lt;= pLink-&gt;MessageSignVector[m].<span class="pl-smi">EndDayNo</span> &amp;&amp;</td>
      </tr>
      <tr>
        <td id="L2282" class="blob-num js-line-number" data-line-number="2282"></td>
        <td id="LC2282" class="blob-code blob-code-inner js-file-line">					CurrentTime &gt;= pLink-&gt;MessageSignVector[m].<span class="pl-smi">StartTime</span> &amp;&amp;</td>
      </tr>
      <tr>
        <td id="L2283" class="blob-num js-line-number" data-line-number="2283"></td>
        <td id="LC2283" class="blob-code blob-code-inner js-file-line">					CurrentTime &lt;= pLink-&gt;MessageSignVector[m].<span class="pl-smi">EndTime</span>)</td>
      </tr>
      <tr>
        <td id="L2284" class="blob-num js-line-number" data-line-number="2284"></td>
        <td id="LC2284" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L2285" class="blob-num js-line-number" data-line-number="2285"></td>
        <td id="LC2285" class="blob-code blob-code-inner js-file-line">					<span class="pl-c">//calculate shortest path</span></td>
      </tr>
      <tr>
        <td id="L2286" class="blob-num js-line-number" data-line-number="2286"></td>
        <td id="LC2286" class="blob-code blob-code-inner js-file-line">					DTANetworkForSP <span class="pl-smi">network</span>(g_NodeVector.<span class="pl-c1">size</span>(), g_LinkVector.<span class="pl-c1">size</span>(), <span class="pl-c1">1</span>, g_AdjLinkSize);</td>
      </tr>
      <tr>
        <td id="L2287" class="blob-num js-line-number" data-line-number="2287"></td>
        <td id="LC2287" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">float</span> COV_perception_erorr = g_VMSPerceptionErrorRatio;</td>
      </tr>
      <tr>
        <td id="L2288" class="blob-num js-line-number" data-line-number="2288"></td>
        <td id="LC2288" class="blob-code blob-code-inner js-file-line">					network.<span class="pl-c1">BuildTravelerInfoNetwork</span>(DayNo, CurrentTime, COV_perception_erorr);</td>
      </tr>
      <tr>
        <td id="L2289" class="blob-num js-line-number" data-line-number="2289"></td>
        <td id="LC2289" class="blob-code blob-code-inner js-file-line">					network.<span class="pl-c1">TDLabelCorrecting_DoubleQueue</span>(g_LinkVector[li]-&gt;m_ToNodeID, <span class="pl-c1">0</span>, CurrentTime, <span class="pl-c1">1</span>, <span class="pl-c1">10</span>, <span class="pl-c1">false</span>, <span class="pl-c1">false</span>, <span class="pl-c1">false</span>);</td>
      </tr>
      <tr>
        <td id="L2290" class="blob-num js-line-number" data-line-number="2290"></td>
        <td id="LC2290" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2291" class="blob-num js-line-number" data-line-number="2291"></td>
        <td id="LC2291" class="blob-code blob-code-inner js-file-line">					<span class="pl-c1">TRACE</span>(<span class="pl-s"><span class="pl-pds">&quot;</span><span class="pl-cce">\n</span>VMS <span class="pl-c1">%d</span> -&gt; <span class="pl-c1">%d</span><span class="pl-pds">&quot;</span></span>, g_LinkVector[li]-&gt;m_FromNodeNumber, pLink-&gt;m_ToNodeNumber);</td>
      </tr>
      <tr>
        <td id="L2292" class="blob-num js-line-number" data-line-number="2292"></td>
        <td id="LC2292" class="blob-code blob-code-inner js-file-line">					<span class="pl-c">// update node predecessor</span></td>
      </tr>
      <tr>
        <td id="L2293" class="blob-num js-line-number" data-line-number="2293"></td>
        <td id="LC2293" class="blob-code blob-code-inner js-file-line">					g_LinkVector[li]-&gt;MessageSignVector[m].<span class="pl-c1">Initialize</span>(g_NodeVector.<span class="pl-c1">size</span>(), network.<span class="pl-smi">NodePredAry</span>, network.<span class="pl-smi">LinkNoAry</span>);</td>
      </tr>
      <tr>
        <td id="L2294" class="blob-num js-line-number" data-line-number="2294"></td>
        <td id="LC2294" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2295" class="blob-num js-line-number" data-line-number="2295"></td>
        <td id="LC2295" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L2296" class="blob-num js-line-number" data-line-number="2296"></td>
        <td id="LC2296" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L2297" class="blob-num js-line-number" data-line-number="2297"></td>
        <td id="LC2297" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L2298" class="blob-num js-line-number" data-line-number="2298"></td>
        <td id="LC2298" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2299" class="blob-num js-line-number" data-line-number="2299"></td>
        <td id="LC2299" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L2300" class="blob-num js-line-number" data-line-number="2300"></td>
        <td id="LC2300" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">// load vehicle into network</span></td>
      </tr>
      <tr>
        <td id="L2301" class="blob-num js-line-number" data-line-number="2301"></td>
        <td id="LC2301" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2302" class="blob-num js-line-number" data-line-number="2302"></td>
        <td id="LC2302" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2303" class="blob-num js-line-number" data-line-number="2303"></td>
        <td id="LC2303" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2304" class="blob-num js-line-number" data-line-number="2304"></td>
        <td id="LC2304" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">// step 1: scan all the vehicles, if a vehicle&#39;s start time &gt;= CurrentTime, and there is available space in the first link,</span></td>
      </tr>
      <tr>
        <td id="L2305" class="blob-num js-line-number" data-line-number="2305"></td>
        <td id="LC2305" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">// load this vehicle into the ready queue</span></td>
      </tr>
      <tr>
        <td id="L2306" class="blob-num js-line-number" data-line-number="2306"></td>
        <td id="LC2306" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2307" class="blob-num js-line-number" data-line-number="2307"></td>
        <td id="LC2307" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">// comment: we use map here as the g_VehicleMap map is sorted by departure time.</span></td>
      </tr>
      <tr>
        <td id="L2308" class="blob-num js-line-number" data-line-number="2308"></td>
        <td id="LC2308" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">// At each iteration, we start  the last loaded id, and exit if the departure time of a vehicle is later than the current time.</span></td>
      </tr>
      <tr>
        <td id="L2309" class="blob-num js-line-number" data-line-number="2309"></td>
        <td id="LC2309" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2310" class="blob-num js-line-number" data-line-number="2310"></td>
        <td id="LC2310" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">for</span> (iterVM = g_VehicleMap.<span class="pl-c1">find</span>(g_LastLoadedVehicleID); iterVM != g_VehicleMap.<span class="pl-c1">end</span>(); iterVM++)</td>
      </tr>
      <tr>
        <td id="L2311" class="blob-num js-line-number" data-line-number="2311"></td>
        <td id="LC2311" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L2312" class="blob-num js-line-number" data-line-number="2312"></td>
        <td id="LC2312" class="blob-code blob-code-inner js-file-line">		DTAVehicle* pVeh = iterVM-&gt;second;</td>
      </tr>
      <tr>
        <td id="L2313" class="blob-num js-line-number" data-line-number="2313"></td>
        <td id="LC2313" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">if</span> (pVeh-&gt;m_bLoaded == <span class="pl-c1">false</span> &amp;&amp; <span class="pl-c1">g_floating_point_value_less_than_or_eq_comparison</span>(pVeh-&gt;m_DepartureTime, CurrentTime))  <span class="pl-c">// not being loaded</span></td>
      </tr>
      <tr>
        <td id="L2314" class="blob-num js-line-number" data-line-number="2314"></td>
        <td id="LC2314" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L2315" class="blob-num js-line-number" data-line-number="2315"></td>
        <td id="LC2315" class="blob-code blob-code-inner js-file-line">		</td>
      </tr>
      <tr>
        <td id="L2316" class="blob-num js-line-number" data-line-number="2316"></td>
        <td id="LC2316" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (pVeh-&gt;m_NodeSize &gt;= <span class="pl-c1">2</span>)  <span class="pl-c">// with physical path</span></td>
      </tr>
      <tr>
        <td id="L2317" class="blob-num js-line-number" data-line-number="2317"></td>
        <td id="LC2317" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L2318" class="blob-num js-line-number" data-line-number="2318"></td>
        <td id="LC2318" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">int</span> FirstLink = pVeh-&gt;m_LinkAry[<span class="pl-c1">0</span>].<span class="pl-smi">LinkNo</span>;</td>
      </tr>
      <tr>
        <td id="L2319" class="blob-num js-line-number" data-line-number="2319"></td>
        <td id="LC2319" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2320" class="blob-num js-line-number" data-line-number="2320"></td>
        <td id="LC2320" class="blob-code blob-code-inner js-file-line">				DTALink* p_link = g_LinkVector[FirstLink];</td>
      </tr>
      <tr>
        <td id="L2321" class="blob-num js-line-number" data-line-number="2321"></td>
        <td id="LC2321" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2322" class="blob-num js-line-number" data-line-number="2322"></td>
        <td id="LC2322" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (p_link != <span class="pl-c1">NULL</span>)</td>
      </tr>
      <tr>
        <td id="L2323" class="blob-num js-line-number" data-line-number="2323"></td>
        <td id="LC2323" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L2324" class="blob-num js-line-number" data-line-number="2324"></td>
        <td id="LC2324" class="blob-code blob-code-inner js-file-line">					struc_vehicle_item vi;</td>
      </tr>
      <tr>
        <td id="L2325" class="blob-num js-line-number" data-line-number="2325"></td>
        <td id="LC2325" class="blob-code blob-code-inner js-file-line">					vi.<span class="pl-smi">veh_id</span> = pVeh-&gt;m_AgentID;</td>
      </tr>
      <tr>
        <td id="L2326" class="blob-num js-line-number" data-line-number="2326"></td>
        <td id="LC2326" class="blob-code blob-code-inner js-file-line">					g_LastLoadedVehicleID = pVeh-&gt;m_AgentID;</td>
      </tr>
      <tr>
        <td id="L2327" class="blob-num js-line-number" data-line-number="2327"></td>
        <td id="LC2327" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2328" class="blob-num js-line-number" data-line-number="2328"></td>
        <td id="LC2328" class="blob-code blob-code-inner js-file-line">					vi.<span class="pl-smi">event_time_stamp</span> = pVeh-&gt;m_DepartureTime + p_link-&gt;<span class="pl-c1">GetFreeMovingTravelTime</span>(TrafficFlowModelFlag, CurrentTime);  <span class="pl-c">// unit: min</span></td>
      </tr>
      <tr>
        <td id="L2329" class="blob-num js-line-number" data-line-number="2329"></td>
        <td id="LC2329" class="blob-code blob-code-inner js-file-line">					pVeh-&gt;m_bLoaded = <span class="pl-c1">true</span>;</td>
      </tr>
      <tr>
        <td id="L2330" class="blob-num js-line-number" data-line-number="2330"></td>
        <td id="LC2330" class="blob-code blob-code-inner js-file-line">					pVeh-&gt;m_SimLinkSequenceNo = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L2331" class="blob-num js-line-number" data-line-number="2331"></td>
        <td id="LC2331" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2332" class="blob-num js-line-number" data-line-number="2332"></td>
        <td id="LC2332" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">if</span> (debug_flag &amp;&amp; vi.<span class="pl-smi">veh_id</span> == vehicle_id_trace)</td>
      </tr>
      <tr>
        <td id="L2333" class="blob-num js-line-number" data-line-number="2333"></td>
        <td id="LC2333" class="blob-code blob-code-inner js-file-line">						<span class="pl-c1">TRACE</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Step 1: Load vhc <span class="pl-c1">%d</span> to link <span class="pl-c1">%d</span> with departure time time <span class="pl-c1">%5.2f</span> -&gt; <span class="pl-c1">%5.2f</span><span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>, vi.<span class="pl-smi">veh_id</span>, FirstLink, pVeh-&gt;m_DepartureTime, vi.<span class="pl-smi">event_time_stamp</span>);</td>
      </tr>
      <tr>
        <td id="L2334" class="blob-num js-line-number" data-line-number="2334"></td>
        <td id="LC2334" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2335" class="blob-num js-line-number" data-line-number="2335"></td>
        <td id="LC2335" class="blob-code blob-code-inner js-file-line">					p_link-&gt;LoadingBuffer.<span class="pl-c1">push_back</span>(vi);  <span class="pl-c">// need to fine-tune</span></td>
      </tr>
      <tr>
        <td id="L2336" class="blob-num js-line-number" data-line-number="2336"></td>
        <td id="LC2336" class="blob-code blob-code-inner js-file-line">					g_Number_of_GeneratedVehicles += <span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L2337" class="blob-num js-line-number" data-line-number="2337"></td>
        <td id="LC2337" class="blob-code blob-code-inner js-file-line">					g_NetworkMOEAry[time_stamp_in_min].<span class="pl-smi">Flow_in_a_min</span> += <span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L2338" class="blob-num js-line-number" data-line-number="2338"></td>
        <td id="LC2338" class="blob-code blob-code-inner js-file-line">					g_NetworkMOEAry[time_stamp_in_min].<span class="pl-smi">CumulativeInFlow</span> = g_Number_of_GeneratedVehicles;</td>
      </tr>
      <tr>
        <td id="L2339" class="blob-num js-line-number" data-line-number="2339"></td>
        <td id="LC2339" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2340" class="blob-num js-line-number" data-line-number="2340"></td>
        <td id="LC2340" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L2341" class="blob-num js-line-number" data-line-number="2341"></td>
        <td id="LC2341" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2342" class="blob-num js-line-number" data-line-number="2342"></td>
        <td id="LC2342" class="blob-code blob-code-inner js-file-line">			}  <span class="pl-c">// with physical path</span></td>
      </tr>
      <tr>
        <td id="L2343" class="blob-num js-line-number" data-line-number="2343"></td>
        <td id="LC2343" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L2344" class="blob-num js-line-number" data-line-number="2344"></td>
        <td id="LC2344" class="blob-code blob-code-inner js-file-line">			{	<span class="pl-c">//without physical path: skip</span></td>
      </tr>
      <tr>
        <td id="L2345" class="blob-num js-line-number" data-line-number="2345"></td>
        <td id="LC2345" class="blob-code blob-code-inner js-file-line">				g_LastLoadedVehicleID = pVeh-&gt;m_AgentID;</td>
      </tr>
      <tr>
        <td id="L2346" class="blob-num js-line-number" data-line-number="2346"></td>
        <td id="LC2346" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L2347" class="blob-num js-line-number" data-line-number="2347"></td>
        <td id="LC2347" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2348" class="blob-num js-line-number" data-line-number="2348"></td>
        <td id="LC2348" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2349" class="blob-num js-line-number" data-line-number="2349"></td>
        <td id="LC2349" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L2350" class="blob-num js-line-number" data-line-number="2350"></td>
        <td id="LC2350" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2351" class="blob-num js-line-number" data-line-number="2351"></td>
        <td id="LC2351" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">if</span> (<span class="pl-c1">g_floating_point_value_less_than_or_eq_comparison</span>(pVeh-&gt;m_DepartureTime, CurrentTime) == <span class="pl-c1">false</span>)</td>
      </tr>
      <tr>
        <td id="L2352" class="blob-num js-line-number" data-line-number="2352"></td>
        <td id="LC2352" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L2353" class="blob-num js-line-number" data-line-number="2353"></td>
        <td id="LC2353" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">break</span>;</td>
      </tr>
      <tr>
        <td id="L2354" class="blob-num js-line-number" data-line-number="2354"></td>
        <td id="LC2354" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L2355" class="blob-num js-line-number" data-line-number="2355"></td>
        <td id="LC2355" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2356" class="blob-num js-line-number" data-line-number="2356"></td>
        <td id="LC2356" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2357" class="blob-num js-line-number" data-line-number="2357"></td>
        <td id="LC2357" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L2358" class="blob-num js-line-number" data-line-number="2358"></td>
        <td id="LC2358" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2359" class="blob-num js-line-number" data-line-number="2359"></td>
        <td id="LC2359" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">// loading buffer</span></td>
      </tr>
      <tr>
        <td id="L2360" class="blob-num js-line-number" data-line-number="2360"></td>
        <td id="LC2360" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2361" class="blob-num js-line-number" data-line-number="2361"></td>
        <td id="LC2361" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> link_size = g_LinkVector.<span class="pl-c1">size</span>();</td>
      </tr>
      <tr>
        <td id="L2362" class="blob-num js-line-number" data-line-number="2362"></td>
        <td id="LC2362" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">pragma</span> omp parallel for</td>
      </tr>
      <tr>
        <td id="L2363" class="blob-num js-line-number" data-line-number="2363"></td>
        <td id="LC2363" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">for</span> (<span class="pl-k">int</span> li = <span class="pl-c1">0</span>; li&lt; link_size; li++)</td>
      </tr>
      <tr>
        <td id="L2364" class="blob-num js-line-number" data-line-number="2364"></td>
        <td id="LC2364" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L2365" class="blob-num js-line-number" data-line-number="2365"></td>
        <td id="LC2365" class="blob-code blob-code-inner js-file-line">		DTALink * pLink = g_LinkVector[li];</td>
      </tr>
      <tr>
        <td id="L2366" class="blob-num js-line-number" data-line-number="2366"></td>
        <td id="LC2366" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2367" class="blob-num js-line-number" data-line-number="2367"></td>
        <td id="LC2367" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">while</span> (pLink-&gt;LoadingBuffer.<span class="pl-c1">size</span>() &gt;<span class="pl-c1">0</span> &amp;&amp; pLink-&gt;<span class="pl-c1">GetNumberOfLanes</span>(DayNo, CurrentTime)&gt;<span class="pl-c1">0.01</span>)  <span class="pl-c">// no load vehicle into a blocked link</span></td>
      </tr>
      <tr>
        <td id="L2368" class="blob-num js-line-number" data-line-number="2368"></td>
        <td id="LC2368" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L2369" class="blob-num js-line-number" data-line-number="2369"></td>
        <td id="LC2369" class="blob-code blob-code-inner js-file-line">			struc_vehicle_item vi = pLink-&gt;LoadingBuffer.<span class="pl-c1">front</span>();</td>
      </tr>
      <tr>
        <td id="L2370" class="blob-num js-line-number" data-line-number="2370"></td>
        <td id="LC2370" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">// we change the time stamp here to reflect the actual loading time into the network, especially for blocked link</span></td>
      </tr>
      <tr>
        <td id="L2371" class="blob-num js-line-number" data-line-number="2371"></td>
        <td id="LC2371" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (vi.<span class="pl-smi">event_time_stamp</span> &lt; CurrentTime)</td>
      </tr>
      <tr>
        <td id="L2372" class="blob-num js-line-number" data-line-number="2372"></td>
        <td id="LC2372" class="blob-code blob-code-inner js-file-line">				vi.<span class="pl-smi">event_time_stamp</span> = CurrentTime;</td>
      </tr>
      <tr>
        <td id="L2373" class="blob-num js-line-number" data-line-number="2373"></td>
        <td id="LC2373" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2374" class="blob-num js-line-number" data-line-number="2374"></td>
        <td id="LC2374" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (debug_flag &amp;&amp; vi.<span class="pl-smi">veh_id</span> == vehicle_id_trace)</td>
      </tr>
      <tr>
        <td id="L2375" class="blob-num js-line-number" data-line-number="2375"></td>
        <td id="LC2375" class="blob-code blob-code-inner js-file-line">				<span class="pl-c1">TRACE</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Step 1: Time <span class="pl-c1">%f</span>: Load vhc <span class="pl-c1">%d</span> from buffer to physical link <span class="pl-c1">%d</span>-&gt;<span class="pl-c1">%d</span><span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>, CurrentTime, vi.<span class="pl-smi">veh_id</span>, pLink-&gt;m_FromNodeNumber, pLink-&gt;m_ToNodeNumber);</td>
      </tr>
      <tr>
        <td id="L2376" class="blob-num js-line-number" data-line-number="2376"></td>
        <td id="LC2376" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2377" class="blob-num js-line-number" data-line-number="2377"></td>
        <td id="LC2377" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">int</span> NumberOfVehiclesOnThisLinkAtCurrentTime = (<span class="pl-k">int</span>)(pLink-&gt;EntranceQueue.<span class="pl-c1">size</span>() + pLink-&gt;ExitQueue.<span class="pl-c1">size</span>());</td>
      </tr>
      <tr>
        <td id="L2378" class="blob-num js-line-number" data-line-number="2378"></td>
        <td id="LC2378" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2379" class="blob-num js-line-number" data-line-number="2379"></td>
        <td id="LC2379" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">// determine link in capacity </span></td>
      </tr>
      <tr>
        <td id="L2380" class="blob-num js-line-number" data-line-number="2380"></td>
        <td id="LC2380" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">float</span> AvailableSpaceCapacity = pLink-&gt;m_VehicleSpaceCapacity - NumberOfVehiclesOnThisLinkAtCurrentTime;</td>
      </tr>
      <tr>
        <td id="L2381" class="blob-num js-line-number" data-line-number="2381"></td>
        <td id="LC2381" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2382" class="blob-num js-line-number" data-line-number="2382"></td>
        <td id="LC2382" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">// if we use BPR function, no density constraint is imposed --&gt; TrafficFlowModelFlag == 0</span></td>
      </tr>
      <tr>
        <td id="L2383" class="blob-num js-line-number" data-line-number="2383"></td>
        <td id="LC2383" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (TrafficFlowModelFlag == <span class="pl-c1">0</span> || AvailableSpaceCapacity &gt;= <span class="pl-c1">max</span>(<span class="pl-c1">2</span>, pLink-&gt;m_VehicleSpaceCapacity*(<span class="pl-c1">1</span> - g_MaxDensityRatioForVehicleLoading)))  <span class="pl-c">// at least 10% remaining capacity or 2 vehicle space is left</span></td>
      </tr>
      <tr>
        <td id="L2384" class="blob-num js-line-number" data-line-number="2384"></td>
        <td id="LC2384" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L2385" class="blob-num js-line-number" data-line-number="2385"></td>
        <td id="LC2385" class="blob-code blob-code-inner js-file-line">				pLink-&gt;LoadingBuffer.<span class="pl-c1">pop_front</span>();</td>
      </tr>
      <tr>
        <td id="L2386" class="blob-num js-line-number" data-line-number="2386"></td>
        <td id="LC2386" class="blob-code blob-code-inner js-file-line">				<span class="pl-c1">ASSERT</span>(vi.<span class="pl-smi">veh_id</span> &gt;= <span class="pl-c1">0</span>);</td>
      </tr>
      <tr>
        <td id="L2387" class="blob-num js-line-number" data-line-number="2387"></td>
        <td id="LC2387" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2388" class="blob-num js-line-number" data-line-number="2388"></td>
        <td id="LC2388" class="blob-code blob-code-inner js-file-line">				pLink-&gt;EntranceQueue.<span class="pl-c1">push_back</span>(vi);</td>
      </tr>
      <tr>
        <td id="L2389" class="blob-num js-line-number" data-line-number="2389"></td>
        <td id="LC2389" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2390" class="blob-num js-line-number" data-line-number="2390"></td>
        <td id="LC2390" class="blob-code blob-code-inner js-file-line">				pLink-&gt;CFlowArrivalCount += <span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L2391" class="blob-num js-line-number" data-line-number="2391"></td>
        <td id="LC2391" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2392" class="blob-num js-line-number" data-line-number="2392"></td>
        <td id="LC2392" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2393" class="blob-num js-line-number" data-line-number="2393"></td>
        <td id="LC2393" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">int</span> demand_type = g_VehicleMap[vi.<span class="pl-smi">veh_id</span>]-&gt;m_DemandType;</td>
      </tr>
      <tr>
        <td id="L2394" class="blob-num js-line-number" data-line-number="2394"></td>
        <td id="LC2394" class="blob-code blob-code-inner js-file-line">				pLink-&gt;CFlowArrivalCount_DemandType[demand_type] += <span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L2395" class="blob-num js-line-number" data-line-number="2395"></td>
        <td id="LC2395" class="blob-code blob-code-inner js-file-line">				pLink-&gt;CFlowArrivalRevenue_DemandType[demand_type] += pLink-&gt;<span class="pl-c1">GetTollRateInDollar</span>(DayNo, CurrentTime, demand_type);</td>
      </tr>
      <tr>
        <td id="L2396" class="blob-num js-line-number" data-line-number="2396"></td>
        <td id="LC2396" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2397" class="blob-num js-line-number" data-line-number="2397"></td>
        <td id="LC2397" class="blob-code blob-code-inner js-file-line">				DTAVehicle* pVehicle = g_VehicleMap[vi.<span class="pl-smi">veh_id</span>];</td>
      </tr>
      <tr>
        <td id="L2398" class="blob-num js-line-number" data-line-number="2398"></td>
        <td id="LC2398" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2399" class="blob-num js-line-number" data-line-number="2399"></td>
        <td id="LC2399" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">// add cumulative flow count to vehicle</span></td>
      </tr>
      <tr>
        <td id="L2400" class="blob-num js-line-number" data-line-number="2400"></td>
        <td id="LC2400" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2401" class="blob-num js-line-number" data-line-number="2401"></td>
        <td id="LC2401" class="blob-code blob-code-inner js-file-line">				pVehicle-&gt;m_TollDollarCost += pLink-&gt;<span class="pl-c1">GetTollRateInDollar</span>(DayNo, CurrentTime, demand_type);</td>
      </tr>
      <tr>
        <td id="L2402" class="blob-num js-line-number" data-line-number="2402"></td>
        <td id="LC2402" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2403" class="blob-num js-line-number" data-line-number="2403"></td>
        <td id="LC2403" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2404" class="blob-num js-line-number" data-line-number="2404"></td>
        <td id="LC2404" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (debug_flag &amp;&amp; vi.<span class="pl-smi">veh_id</span> == vehicle_id_trace)</td>
      </tr>
      <tr>
        <td id="L2405" class="blob-num js-line-number" data-line-number="2405"></td>
        <td id="LC2405" class="blob-code blob-code-inner js-file-line">					<span class="pl-c1">TRACE</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Step 1: Time <span class="pl-c1">%f</span>: Capacity available, remove vhc <span class="pl-c1">%d</span> from buffer to physical link <span class="pl-c1">%d</span>-&gt;<span class="pl-c1">%d</span><span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>, CurrentTime, vi.<span class="pl-smi">veh_id</span>, pLink-&gt;m_FromNodeNumber, pLink-&gt;m_ToNodeNumber);</td>
      </tr>
      <tr>
        <td id="L2406" class="blob-num js-line-number" data-line-number="2406"></td>
        <td id="LC2406" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2407" class="blob-num js-line-number" data-line-number="2407"></td>
        <td id="LC2407" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L2408" class="blob-num js-line-number" data-line-number="2408"></td>
        <td id="LC2408" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L2409" class="blob-num js-line-number" data-line-number="2409"></td>
        <td id="LC2409" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L2410" class="blob-num js-line-number" data-line-number="2410"></td>
        <td id="LC2410" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">break</span>;  <span class="pl-c">// physical road is too congested, wait for next time interval to load</span></td>
      </tr>
      <tr>
        <td id="L2411" class="blob-num js-line-number" data-line-number="2411"></td>
        <td id="LC2411" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L2412" class="blob-num js-line-number" data-line-number="2412"></td>
        <td id="LC2412" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2413" class="blob-num js-line-number" data-line-number="2413"></td>
        <td id="LC2413" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L2414" class="blob-num js-line-number" data-line-number="2414"></td>
        <td id="LC2414" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L2415" class="blob-num js-line-number" data-line-number="2415"></td>
        <td id="LC2415" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2416" class="blob-num js-line-number" data-line-number="2416"></td>
        <td id="LC2416" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">// step 2: move vehicles from EntranceQueue To ExitQueue, if ReadyTime &lt;= CurrentTime)</span></td>
      </tr>
      <tr>
        <td id="L2417" class="blob-num js-line-number" data-line-number="2417"></td>
        <td id="LC2417" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2418" class="blob-num js-line-number" data-line-number="2418"></td>
        <td id="LC2418" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">pragma</span> omp parallel for</td>
      </tr>
      <tr>
        <td id="L2419" class="blob-num js-line-number" data-line-number="2419"></td>
        <td id="LC2419" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">for</span> (<span class="pl-k">int</span> li = <span class="pl-c1">0</span>; li&lt; link_size; li++)</td>
      </tr>
      <tr>
        <td id="L2420" class="blob-num js-line-number" data-line-number="2420"></td>
        <td id="LC2420" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L2421" class="blob-num js-line-number" data-line-number="2421"></td>
        <td id="LC2421" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2422" class="blob-num js-line-number" data-line-number="2422"></td>
        <td id="LC2422" class="blob-code blob-code-inner js-file-line">		DTALink * pLink = g_LinkVector[li];</td>
      </tr>
      <tr>
        <td id="L2423" class="blob-num js-line-number" data-line-number="2423"></td>
        <td id="LC2423" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2424" class="blob-num js-line-number" data-line-number="2424"></td>
        <td id="LC2424" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">while</span> (pLink-&gt;EntranceQueue.<span class="pl-c1">size</span>() &gt;<span class="pl-c1">0</span>)</td>
      </tr>
      <tr>
        <td id="L2425" class="blob-num js-line-number" data-line-number="2425"></td>
        <td id="LC2425" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L2426" class="blob-num js-line-number" data-line-number="2426"></td>
        <td id="LC2426" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2427" class="blob-num js-line-number" data-line-number="2427"></td>
        <td id="LC2427" class="blob-code blob-code-inner js-file-line">			struc_vehicle_item vi = pLink-&gt;EntranceQueue.<span class="pl-c1">front</span>();</td>
      </tr>
      <tr>
        <td id="L2428" class="blob-num js-line-number" data-line-number="2428"></td>
        <td id="LC2428" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">double</span> PlannedArrivalTime = vi.<span class="pl-smi">event_time_stamp</span>;</td>
      </tr>
      <tr>
        <td id="L2429" class="blob-num js-line-number" data-line-number="2429"></td>
        <td id="LC2429" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2430" class="blob-num js-line-number" data-line-number="2430"></td>
        <td id="LC2430" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (debug_flag &amp;&amp; pLink-&gt;m_FromNodeNumber == <span class="pl-c1">34</span> &amp;&amp; pLink-&gt;m_ToNodeNumber == <span class="pl-c1">30</span> &amp;&amp; CurrentTime &gt;= <span class="pl-c1">860</span>)</td>
      </tr>
      <tr>
        <td id="L2431" class="blob-num js-line-number" data-line-number="2431"></td>
        <td id="LC2431" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L2432" class="blob-num js-line-number" data-line-number="2432"></td>
        <td id="LC2432" class="blob-code blob-code-inner js-file-line">				<span class="pl-c1">TRACE</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Step 3: Time <span class="pl-c1">%f</span>, Link: <span class="pl-c1">%d</span> -&gt; <span class="pl-c1">%d</span>: entrance queue length: <span class="pl-c1">%d</span>, exit queue length <span class="pl-c1">%d</span><span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>,</td>
      </tr>
      <tr>
        <td id="L2433" class="blob-num js-line-number" data-line-number="2433"></td>
        <td id="LC2433" class="blob-code blob-code-inner js-file-line">					CurrentTime, pLink-&gt;m_FromNodeNumber, pLink-&gt;m_ToNodeNumber,</td>
      </tr>
      <tr>
        <td id="L2434" class="blob-num js-line-number" data-line-number="2434"></td>
        <td id="LC2434" class="blob-code blob-code-inner js-file-line">					pLink-&gt;EntranceQueue.<span class="pl-c1">size</span>(), pLink-&gt;ExitQueue.<span class="pl-c1">size</span>());</td>
      </tr>
      <tr>
        <td id="L2435" class="blob-num js-line-number" data-line-number="2435"></td>
        <td id="LC2435" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L2436" class="blob-num js-line-number" data-line-number="2436"></td>
        <td id="LC2436" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2437" class="blob-num js-line-number" data-line-number="2437"></td>
        <td id="LC2437" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (<span class="pl-c1">g_floating_point_value_less_than_or_eq_comparison</span>(PlannedArrivalTime, CurrentTime))  <span class="pl-c">// link arrival time within the simulation time interval</span></td>
      </tr>
      <tr>
        <td id="L2438" class="blob-num js-line-number" data-line-number="2438"></td>
        <td id="LC2438" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L2439" class="blob-num js-line-number" data-line-number="2439"></td>
        <td id="LC2439" class="blob-code blob-code-inner js-file-line">				pLink-&gt;EntranceQueue.<span class="pl-c1">pop_front</span>();</td>
      </tr>
      <tr>
        <td id="L2440" class="blob-num js-line-number" data-line-number="2440"></td>
        <td id="LC2440" class="blob-code blob-code-inner js-file-line">				pLink-&gt;ExitQueue.<span class="pl-c1">push_back</span>(vi);</td>
      </tr>
      <tr>
        <td id="L2441" class="blob-num js-line-number" data-line-number="2441"></td>
        <td id="LC2441" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2442" class="blob-num js-line-number" data-line-number="2442"></td>
        <td id="LC2442" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (debug_flag &amp;&amp; vi.<span class="pl-smi">veh_id</span> == vehicle_id_trace)</td>
      </tr>
      <tr>
        <td id="L2443" class="blob-num js-line-number" data-line-number="2443"></td>
        <td id="LC2443" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L2444" class="blob-num js-line-number" data-line-number="2444"></td>
        <td id="LC2444" class="blob-code blob-code-inner js-file-line">					<span class="pl-c1">TRACE</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Step 2: Time <span class="pl-c1">%f</span>: Vhc <span class="pl-c1">%d</span> moves from entrance queue to exit queue on link <span class="pl-c1">%d</span>-&gt;<span class="pl-c1">%d</span><span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>, PlannedArrivalTime, vi.<span class="pl-smi">veh_id</span>, g_LinkVector[li]-&gt;m_FromNodeNumber, g_LinkVector[li]-&gt;m_ToNodeNumber);</td>
      </tr>
      <tr>
        <td id="L2445" class="blob-num js-line-number" data-line-number="2445"></td>
        <td id="LC2445" class="blob-code blob-code-inner js-file-line">					<span class="pl-c">//					link_id_trace = li;</span></td>
      </tr>
      <tr>
        <td id="L2446" class="blob-num js-line-number" data-line-number="2446"></td>
        <td id="LC2446" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L2447" class="blob-num js-line-number" data-line-number="2447"></td>
        <td id="LC2447" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2448" class="blob-num js-line-number" data-line-number="2448"></td>
        <td id="LC2448" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L2449" class="blob-num js-line-number" data-line-number="2449"></td>
        <td id="LC2449" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L2450" class="blob-num js-line-number" data-line-number="2450"></td>
        <td id="LC2450" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L2451" class="blob-num js-line-number" data-line-number="2451"></td>
        <td id="LC2451" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">break</span>;  <span class="pl-c">// the vehicle&#39;s actual arrival time is later than the current time, so we exit from the loop, stop searching</span></td>
      </tr>
      <tr>
        <td id="L2452" class="blob-num js-line-number" data-line-number="2452"></td>
        <td id="LC2452" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L2453" class="blob-num js-line-number" data-line-number="2453"></td>
        <td id="LC2453" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2454" class="blob-num js-line-number" data-line-number="2454"></td>
        <td id="LC2454" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L2455" class="blob-num js-line-number" data-line-number="2455"></td>
        <td id="LC2455" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L2456" class="blob-num js-line-number" data-line-number="2456"></td>
        <td id="LC2456" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2457" class="blob-num js-line-number" data-line-number="2457"></td>
        <td id="LC2457" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">// step 3: determine link in and out capacity</span></td>
      </tr>
      <tr>
        <td id="L2458" class="blob-num js-line-number" data-line-number="2458"></td>
        <td id="LC2458" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2459" class="blob-num js-line-number" data-line-number="2459"></td>
        <td id="LC2459" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">if</span> (TrafficFlowModelFlag == <span class="pl-c1">0</span>) <span class="pl-c">// BPR function </span></td>
      </tr>
      <tr>
        <td id="L2460" class="blob-num js-line-number" data-line-number="2460"></td>
        <td id="LC2460" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L2461" class="blob-num js-line-number" data-line-number="2461"></td>
        <td id="LC2461" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">for</span> (<span class="pl-k">unsigned</span> li = <span class="pl-c1">0</span>; li&lt; g_LinkVector.<span class="pl-c1">size</span>(); li++)</td>
      </tr>
      <tr>
        <td id="L2462" class="blob-num js-line-number" data-line-number="2462"></td>
        <td id="LC2462" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L2463" class="blob-num js-line-number" data-line-number="2463"></td>
        <td id="LC2463" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2464" class="blob-num js-line-number" data-line-number="2464"></td>
        <td id="LC2464" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">// under BPR function, we do not impose the physical capacity constraint but use a BPR link travel time to move vehicles along links	</span></td>
      </tr>
      <tr>
        <td id="L2465" class="blob-num js-line-number" data-line-number="2465"></td>
        <td id="LC2465" class="blob-code blob-code-inner js-file-line">			g_LinkVector[li]-&gt;LinkOutCapacity = <span class="pl-c1">99999</span>;</td>
      </tr>
      <tr>
        <td id="L2466" class="blob-num js-line-number" data-line-number="2466"></td>
        <td id="LC2466" class="blob-code blob-code-inner js-file-line">			g_LinkVector[li]-&gt;LinkInCapacity = <span class="pl-c1">99999</span>;</td>
      </tr>
      <tr>
        <td id="L2467" class="blob-num js-line-number" data-line-number="2467"></td>
        <td id="LC2467" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2468" class="blob-num js-line-number" data-line-number="2468"></td>
        <td id="LC2468" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L2469" class="blob-num js-line-number" data-line-number="2469"></td>
        <td id="LC2469" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L2470" class="blob-num js-line-number" data-line-number="2470"></td>
        <td id="LC2470" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">else</span> <span class="pl-c">// queueing model</span></td>
      </tr>
      <tr>
        <td id="L2471" class="blob-num js-line-number" data-line-number="2471"></td>
        <td id="LC2471" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L2472" class="blob-num js-line-number" data-line-number="2472"></td>
        <td id="LC2472" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2473" class="blob-num js-line-number" data-line-number="2473"></td>
        <td id="LC2473" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2474" class="blob-num js-line-number" data-line-number="2474"></td>
        <td id="LC2474" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">pragma</span> omp parallel for</td>
      </tr>
      <tr>
        <td id="L2475" class="blob-num js-line-number" data-line-number="2475"></td>
        <td id="LC2475" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">for</span> (<span class="pl-k">int</span> li = <span class="pl-c1">0</span>; li&lt; link_size; li++)</td>
      </tr>
      <tr>
        <td id="L2476" class="blob-num js-line-number" data-line-number="2476"></td>
        <td id="LC2476" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L2477" class="blob-num js-line-number" data-line-number="2477"></td>
        <td id="LC2477" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2478" class="blob-num js-line-number" data-line-number="2478"></td>
        <td id="LC2478" class="blob-code blob-code-inner js-file-line">			DTALink* pLink = g_LinkVector[li];</td>
      </tr>
      <tr>
        <td id="L2479" class="blob-num js-line-number" data-line-number="2479"></td>
        <td id="LC2479" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">float</span> PerHourCapacity = pLink-&gt;<span class="pl-c1">GetHourlyPerLaneCapacity</span>(CurrentTime);  <span class="pl-c">// static capacity from BRP function</span></td>
      </tr>
      <tr>
        <td id="L2480" class="blob-num js-line-number" data-line-number="2480"></td>
        <td id="LC2480" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">float</span> PerHourCapacityAtCurrentSimulatioInterval = PerHourCapacity;</td>
      </tr>
      <tr>
        <td id="L2481" class="blob-num js-line-number" data-line-number="2481"></td>
        <td id="LC2481" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2482" class="blob-num js-line-number" data-line-number="2482"></td>
        <td id="LC2482" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">// freeway </span></td>
      </tr>
      <tr>
        <td id="L2483" class="blob-num js-line-number" data-line-number="2483"></td>
        <td id="LC2483" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2484" class="blob-num js-line-number" data-line-number="2484"></td>
        <td id="LC2484" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2485" class="blob-num js-line-number" data-line-number="2485"></td>
        <td id="LC2485" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (g_LinkTypeMap[pLink-&gt;m_link_type].<span class="pl-c1">IsFreeway</span>())</td>
      </tr>
      <tr>
        <td id="L2486" class="blob-num js-line-number" data-line-number="2486"></td>
        <td id="LC2486" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L2487" class="blob-num js-line-number" data-line-number="2487"></td>
        <td id="LC2487" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (g_StochasticCapacityMode &amp;&amp;  pLink-&gt;m_StochaticCapcityFlag &gt;= <span class="pl-c1">1</span> &amp;&amp; meso_simulation_time_interval_no % <span class="pl-c1">150</span> == <span class="pl-c1">0</span>)  <span class="pl-c">// update stochastic capacity every 15 min</span></td>
      </tr>
      <tr>
        <td id="L2488" class="blob-num js-line-number" data-line-number="2488"></td>
        <td id="LC2488" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L2489" class="blob-num js-line-number" data-line-number="2489"></td>
        <td id="LC2489" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">bool</span> QueueFlag = <span class="pl-c1">false</span>;</td>
      </tr>
      <tr>
        <td id="L2490" class="blob-num js-line-number" data-line-number="2490"></td>
        <td id="LC2490" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2491" class="blob-num js-line-number" data-line-number="2491"></td>
        <td id="LC2491" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">if</span> (pLink-&gt;ExitQueue.<span class="pl-c1">size</span>() &gt; <span class="pl-c1">0</span>)  <span class="pl-c">// vertical exit queue</span></td>
      </tr>
      <tr>
        <td id="L2492" class="blob-num js-line-number" data-line-number="2492"></td>
        <td id="LC2492" class="blob-code blob-code-inner js-file-line">						QueueFlag = <span class="pl-c1">true</span>;</td>
      </tr>
      <tr>
        <td id="L2493" class="blob-num js-line-number" data-line-number="2493"></td>
        <td id="LC2493" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2494" class="blob-num js-line-number" data-line-number="2494"></td>
        <td id="LC2494" class="blob-code blob-code-inner js-file-line">					PerHourCapacityAtCurrentSimulatioInterval = <span class="pl-c1">GetStochasticCapacity</span>(QueueFlag, pLink-&gt;<span class="pl-c1">GetHourlyPerLaneCapacity</span>(CurrentTime));</td>
      </tr>
      <tr>
        <td id="L2495" class="blob-num js-line-number" data-line-number="2495"></td>
        <td id="LC2495" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L2496" class="blob-num js-line-number" data-line-number="2496"></td>
        <td id="LC2496" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L2497" class="blob-num js-line-number" data-line-number="2497"></td>
        <td id="LC2497" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2498" class="blob-num js-line-number" data-line-number="2498"></td>
        <td id="LC2498" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2499" class="blob-num js-line-number" data-line-number="2499"></td>
        <td id="LC2499" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2500" class="blob-num js-line-number" data-line-number="2500"></td>
        <td id="LC2500" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">float</span> MaximumFlowRate = PerHourCapacityAtCurrentSimulatioInterval *g_DTASimulationInterval / <span class="pl-c1">60</span>.<span class="pl-c1">0f</span>*pLink-&gt;<span class="pl-c1">GetNumberOfLanes</span>(DayNo, CurrentTime); <span class="pl-c">//60 --&gt; cap per min --&gt; unit # of vehicle per simulation interval</span></td>
      </tr>
      <tr>
        <td id="L2501" class="blob-num js-line-number" data-line-number="2501"></td>
        <td id="LC2501" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2502" class="blob-num js-line-number" data-line-number="2502"></td>
        <td id="LC2502" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">float</span> Capacity = MaximumFlowRate;</td>
      </tr>
      <tr>
        <td id="L2503" class="blob-num js-line-number" data-line-number="2503"></td>
        <td id="LC2503" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">// use integer number of vehicles as unit of capacity</span></td>
      </tr>
      <tr>
        <td id="L2504" class="blob-num js-line-number" data-line-number="2504"></td>
        <td id="LC2504" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2505" class="blob-num js-line-number" data-line-number="2505"></td>
        <td id="LC2505" class="blob-code blob-code-inner js-file-line">			pLink-&gt;LinkOutCapacity = <span class="pl-c1">g_GetRandomInteger_From_FloatingPointValue_BasedOnLinkIDAndTimeStamp</span>(Capacity, li);</td>
      </tr>
      <tr>
        <td id="L2506" class="blob-num js-line-number" data-line-number="2506"></td>
        <td id="LC2506" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2507" class="blob-num js-line-number" data-line-number="2507"></td>
        <td id="LC2507" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (debug_flag &amp;&amp; pLink-&gt;m_FromNodeNumber == <span class="pl-c1">34</span> &amp;&amp; pLink-&gt;m_ToNodeNumber == <span class="pl-c1">30</span> &amp;&amp; CurrentTime &gt;= <span class="pl-c1">860</span>)</td>
      </tr>
      <tr>
        <td id="L2508" class="blob-num js-line-number" data-line-number="2508"></td>
        <td id="LC2508" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L2509" class="blob-num js-line-number" data-line-number="2509"></td>
        <td id="LC2509" class="blob-code blob-code-inner js-file-line">				<span class="pl-c1">TRACE</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Step 3: Time <span class="pl-c1">%f</span>, Link: <span class="pl-c1">%d</span> -&gt; <span class="pl-c1">%d</span>: entrance queue length: <span class="pl-c1">%d</span>, exit queue length <span class="pl-c1">%d</span>, cap = <span class="pl-c1">%f</span>, int, <span class="pl-c1">%d</span><span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>,</td>
      </tr>
      <tr>
        <td id="L2510" class="blob-num js-line-number" data-line-number="2510"></td>
        <td id="LC2510" class="blob-code blob-code-inner js-file-line">					CurrentTime, pLink-&gt;m_FromNodeNumber, pLink-&gt;m_ToNodeNumber,</td>
      </tr>
      <tr>
        <td id="L2511" class="blob-num js-line-number" data-line-number="2511"></td>
        <td id="LC2511" class="blob-code blob-code-inner js-file-line">					pLink-&gt;EntranceQueue.<span class="pl-c1">size</span>(), pLink-&gt;ExitQueue.<span class="pl-c1">size</span>(), Capacity, pLink-&gt;LinkOutCapacity);</td>
      </tr>
      <tr>
        <td id="L2512" class="blob-num js-line-number" data-line-number="2512"></td>
        <td id="LC2512" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L2513" class="blob-num js-line-number" data-line-number="2513"></td>
        <td id="LC2513" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2514" class="blob-num js-line-number" data-line-number="2514"></td>
        <td id="LC2514" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2515" class="blob-num js-line-number" data-line-number="2515"></td>
        <td id="LC2515" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">int</span> NumberOfVehiclesOnThisLinkAtCurrentTime = (<span class="pl-k">int</span>)(pLink-&gt;CFlowArrivalCount - pLink-&gt;CFlowDepartureCount);</td>
      </tr>
      <tr>
        <td id="L2516" class="blob-num js-line-number" data-line-number="2516"></td>
        <td id="LC2516" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2517" class="blob-num js-line-number" data-line-number="2517"></td>
        <td id="LC2517" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">float</span> <span class="pl-smi">fLinkInCapacity</span> = <span class="pl-c1">99999.0</span>;</td>
      </tr>
      <tr>
        <td id="L2518" class="blob-num js-line-number" data-line-number="2518"></td>
        <td id="LC2518" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2519" class="blob-num js-line-number" data-line-number="2519"></td>
        <td id="LC2519" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">// TrafficFlowModelFlag == 1 -&gt; point queue model</span></td>
      </tr>
      <tr>
        <td id="L2520" class="blob-num js-line-number" data-line-number="2520"></td>
        <td id="LC2520" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (TrafficFlowModelFlag &gt;= <span class="pl-c1">2</span>)  <span class="pl-c">// apply spatial link in capacity for spatial queue models( with spillback and shockwave) </span></td>
      </tr>
      <tr>
        <td id="L2521" class="blob-num js-line-number" data-line-number="2521"></td>
        <td id="LC2521" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L2522" class="blob-num js-line-number" data-line-number="2522"></td>
        <td id="LC2522" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">// determine link in capacity </span></td>
      </tr>
      <tr>
        <td id="L2523" class="blob-num js-line-number" data-line-number="2523"></td>
        <td id="LC2523" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">float</span> AvailableSpaceCapacity = pLink-&gt;m_VehicleSpaceCapacity - NumberOfVehiclesOnThisLinkAtCurrentTime;</td>
      </tr>
      <tr>
        <td id="L2524" class="blob-num js-line-number" data-line-number="2524"></td>
        <td id="LC2524" class="blob-code blob-code-inner js-file-line">				<span class="pl-smi">fLinkInCapacity</span> = <span class="pl-c1">min</span>(AvailableSpaceCapacity, MaximumFlowRate);</td>
      </tr>
      <tr>
        <td id="L2525" class="blob-num js-line-number" data-line-number="2525"></td>
        <td id="LC2525" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">//			TRACE(&quot; time %5.2f, SC: %5.2f, MFR %5.2f\n&quot;,CurrentTime, AvailableSpaceCapacity, MaximumFlowRate);</span></td>
      </tr>
      <tr>
        <td id="L2526" class="blob-num js-line-number" data-line-number="2526"></td>
        <td id="LC2526" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2527" class="blob-num js-line-number" data-line-number="2527"></td>
        <td id="LC2527" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">// the inflow capcaity is the minimum of (1) incoming maximum flow rate (determined by the number of lanes) and (2) available space capacty  on the link.</span></td>
      </tr>
      <tr>
        <td id="L2528" class="blob-num js-line-number" data-line-number="2528"></td>
        <td id="LC2528" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">// use integer number of vehicles as unit of capacity</span></td>
      </tr>
      <tr>
        <td id="L2529" class="blob-num js-line-number" data-line-number="2529"></td>
        <td id="LC2529" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2530" class="blob-num js-line-number" data-line-number="2530"></td>
        <td id="LC2530" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (TrafficFlowModelFlag == <span class="pl-c1">3</span> &amp;&amp; g_LinkTypeMap[pLink-&gt;m_link_type].<span class="pl-c1">IsFreeway</span>())  <span class="pl-c">// Newell&#39;s model on freeway only</span></td>
      </tr>
      <tr>
        <td id="L2531" class="blob-num js-line-number" data-line-number="2531"></td>
        <td id="LC2531" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L2532" class="blob-num js-line-number" data-line-number="2532"></td>
        <td id="LC2532" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">if</span> (meso_simulation_time_interval_no &gt;= pLink-&gt;m_BackwardWaveTimeInSimulationInterval) <span class="pl-c">/// we only apply backward wave checking after the simulation time is later than the backward wave speed interval</span></td>
      </tr>
      <tr>
        <td id="L2533" class="blob-num js-line-number" data-line-number="2533"></td>
        <td id="LC2533" class="blob-code blob-code-inner js-file-line">					{</td>
      </tr>
      <tr>
        <td id="L2534" class="blob-num js-line-number" data-line-number="2534"></td>
        <td id="LC2534" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">if</span> (debug_flag &amp;&amp; link_id_trace == li &amp;&amp; CurrentTime &gt;= <span class="pl-c1">480</span>)</td>
      </tr>
      <tr>
        <td id="L2535" class="blob-num js-line-number" data-line-number="2535"></td>
        <td id="LC2535" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L2536" class="blob-num js-line-number" data-line-number="2536"></td>
        <td id="LC2536" class="blob-code blob-code-inner js-file-line">							<span class="pl-c1">TRACE</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Step 3: Time <span class="pl-c1">%f</span>, Link: <span class="pl-c1">%d</span> -&gt; <span class="pl-c1">%d</span>: tracing backward wave<span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>,</td>
      </tr>
      <tr>
        <td id="L2537" class="blob-num js-line-number" data-line-number="2537"></td>
        <td id="LC2537" class="blob-code blob-code-inner js-file-line">								CurrentTime, g_NodeVector[pLink-&gt;m_FromNodeID].<span class="pl-smi">m_NodeNumber</span>, g_NodeVector[pLink-&gt;m_ToNodeID].<span class="pl-smi">m_NodeNumber</span>,</td>
      </tr>
      <tr>
        <td id="L2538" class="blob-num js-line-number" data-line-number="2538"></td>
        <td id="LC2538" class="blob-code blob-code-inner js-file-line">								pLink-&gt;EntranceQueue.<span class="pl-c1">size</span>(), pLink-&gt;ExitQueue.<span class="pl-c1">size</span>());</td>
      </tr>
      <tr>
        <td id="L2539" class="blob-num js-line-number" data-line-number="2539"></td>
        <td id="LC2539" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L2540" class="blob-num js-line-number" data-line-number="2540"></td>
        <td id="LC2540" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2541" class="blob-num js-line-number" data-line-number="2541"></td>
        <td id="LC2541" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">int</span> t_residual_minus_backwardwaveTime = <span class="pl-c1">max</span>(<span class="pl-c1">0</span>, meso_simulation_time_interval_no - pLink-&gt;m_BackwardWaveTimeInSimulationInterval) % MAX_TIME_INTERVAL_ADCURVE;</td>
      </tr>
      <tr>
        <td id="L2542" class="blob-num js-line-number" data-line-number="2542"></td>
        <td id="LC2542" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2543" class="blob-num js-line-number" data-line-number="2543"></td>
        <td id="LC2543" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">float</span> VehCountUnderJamDensity = pLink-&gt;m_Length * pLink-&gt;<span class="pl-c1">GetNumberOfLanes</span>(DayNo, CurrentTime) *pLink-&gt;m_KJam;</td>
      </tr>
      <tr>
        <td id="L2544" class="blob-num js-line-number" data-line-number="2544"></td>
        <td id="LC2544" class="blob-code blob-code-inner js-file-line">						<span class="pl-c">// when there is a capacity reduction, in the above model, we assume the space capacity is also reduced proportional to the out flow discharge capacity, </span></td>
      </tr>
      <tr>
        <td id="L2545" class="blob-num js-line-number" data-line-number="2545"></td>
        <td id="LC2545" class="blob-code blob-code-inner js-file-line">						<span class="pl-c">// for example, if the out flow capacity is reduced from 5 lanes to 1 lanes, say 10K vehicles per hour to 2K vehicles per hour, </span></td>
      </tr>
      <tr>
        <td id="L2546" class="blob-num js-line-number" data-line-number="2546"></td>
        <td id="LC2546" class="blob-code blob-code-inner js-file-line">						<span class="pl-c">// our model will also reduce the # of vehicles can be stored on the incident site by the equivalent 4 lanes. </span></td>
      </tr>
      <tr>
        <td id="L2547" class="blob-num js-line-number" data-line-number="2547"></td>
        <td id="LC2547" class="blob-code blob-code-inner js-file-line">						<span class="pl-c">// this might cause the dramatic speed change on the incident site, while the upstream link (with the original space capacity) will take more time to propapate the speed change compared to the incident link. </span></td>
      </tr>
      <tr>
        <td id="L2548" class="blob-num js-line-number" data-line-number="2548"></td>
        <td id="LC2548" class="blob-code blob-code-inner js-file-line">						<span class="pl-c">// to do list: we should add another parameter of space capacity reduction ratio to represent the fact that the space capacity reduction magnitude is different from the outflow capacity reduction level,</span></td>
      </tr>
      <tr>
        <td id="L2549" class="blob-num js-line-number" data-line-number="2549"></td>
        <td id="LC2549" class="blob-code blob-code-inner js-file-line">						<span class="pl-c">// particularly for road construction areas on long links, with smooth barrier on the merging section. </span></td>
      </tr>
      <tr>
        <td id="L2550" class="blob-num js-line-number" data-line-number="2550"></td>
        <td id="LC2550" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">int</span> N_Arrival_Now_Constrainted = (<span class="pl-k">int</span>)(pLink-&gt;m_CumuDeparturelFlow[t_residual_minus_backwardwaveTime] + VehCountUnderJamDensity);  <span class="pl-c">//pLink-&gt;m_Length &#39;s unit is mile</span></td>
      </tr>
      <tr>
        <td id="L2551" class="blob-num js-line-number" data-line-number="2551"></td>
        <td id="LC2551" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">int</span> t_residual_minus_1 = <span class="pl-c1">max</span>(<span class="pl-c1">0</span>, meso_simulation_time_interval_no - <span class="pl-c1">1</span>) % MAX_TIME_INTERVAL_ADCURVE;</td>
      </tr>
      <tr>
        <td id="L2552" class="blob-num js-line-number" data-line-number="2552"></td>
        <td id="LC2552" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2553" class="blob-num js-line-number" data-line-number="2553"></td>
        <td id="LC2553" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">int</span> N_Now_minus_1 = (<span class="pl-k">int</span>)pLink-&gt;m_CumuArrivalFlow[t_residual_minus_1];</td>
      </tr>
      <tr>
        <td id="L2554" class="blob-num js-line-number" data-line-number="2554"></td>
        <td id="LC2554" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">int</span> Flow_allowed = N_Arrival_Now_Constrainted - N_Now_minus_1;</td>
      </tr>
      <tr>
        <td id="L2555" class="blob-num js-line-number" data-line-number="2555"></td>
        <td id="LC2555" class="blob-code blob-code-inner js-file-line">						<span class="pl-c1">TRACE</span>(<span class="pl-s"><span class="pl-pds">&quot;</span><span class="pl-cce">\n</span>time <span class="pl-c1">%d</span>, D:<span class="pl-c1">%d</span>,A<span class="pl-c1">%d</span><span class="pl-pds">&quot;</span></span>, meso_simulation_time_interval_no, pLink-&gt;m_CumuDeparturelFlow[t_residual_minus_1], pLink-&gt;m_CumuArrivalFlow[t_residual_minus_1]);</td>
      </tr>
      <tr>
        <td id="L2556" class="blob-num js-line-number" data-line-number="2556"></td>
        <td id="LC2556" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2557" class="blob-num js-line-number" data-line-number="2557"></td>
        <td id="LC2557" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">if</span> (Flow_allowed &lt; <span class="pl-c1">0</span>)</td>
      </tr>
      <tr>
        <td id="L2558" class="blob-num js-line-number" data-line-number="2558"></td>
        <td id="LC2558" class="blob-code blob-code-inner js-file-line">							Flow_allowed = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L2559" class="blob-num js-line-number" data-line-number="2559"></td>
        <td id="LC2559" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2560" class="blob-num js-line-number" data-line-number="2560"></td>
        <td id="LC2560" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">if</span> (<span class="pl-smi">fLinkInCapacity</span>  &gt; Flow_allowed)</td>
      </tr>
      <tr>
        <td id="L2561" class="blob-num js-line-number" data-line-number="2561"></td>
        <td id="LC2561" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L2562" class="blob-num js-line-number" data-line-number="2562"></td>
        <td id="LC2562" class="blob-code blob-code-inner js-file-line">							<span class="pl-smi">fLinkInCapacity</span> = Flow_allowed;</td>
      </tr>
      <tr>
        <td id="L2563" class="blob-num js-line-number" data-line-number="2563"></td>
        <td id="LC2563" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2564" class="blob-num js-line-number" data-line-number="2564"></td>
        <td id="LC2564" class="blob-code blob-code-inner js-file-line">							<span class="pl-k">if</span> (Flow_allowed == <span class="pl-c1">0</span> &amp;&amp; N_Arrival_Now_Constrainted &gt; <span class="pl-c1">0</span>)</td>
      </tr>
      <tr>
        <td id="L2565" class="blob-num js-line-number" data-line-number="2565"></td>
        <td id="LC2565" class="blob-code blob-code-inner js-file-line">							{</td>
      </tr>
      <tr>
        <td id="L2566" class="blob-num js-line-number" data-line-number="2566"></td>
        <td id="LC2566" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">if</span> (pLink-&gt;m_JamTimeStamp &gt; CurrentTime)</td>
      </tr>
      <tr>
        <td id="L2567" class="blob-num js-line-number" data-line-number="2567"></td>
        <td id="LC2567" class="blob-code blob-code-inner js-file-line">									pLink-&gt;m_JamTimeStamp = CurrentTime;</td>
      </tr>
      <tr>
        <td id="L2568" class="blob-num js-line-number" data-line-number="2568"></td>
        <td id="LC2568" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2569" class="blob-num js-line-number" data-line-number="2569"></td>
        <td id="LC2569" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">//							g_LogFile &lt;&lt; &quot;Queue spillback&quot;&lt;&lt; CurrentTime &lt;&lt;  g_NodeVector[pLink-&gt;m_FromNodeID] &lt;&lt; &quot; -&gt; &quot; &lt;&lt;	g_NodeVector[pLink-&gt;m_ToNodeID] &lt;&lt; &quot; &quot; &lt;&lt; pLink-&gt;LinkInCapacity &lt;&lt; endl;</span></td>
      </tr>
      <tr>
        <td id="L2570" class="blob-num js-line-number" data-line-number="2570"></td>
        <td id="LC2570" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2571" class="blob-num js-line-number" data-line-number="2571"></td>
        <td id="LC2571" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">// update traffic state</span></td>
      </tr>
      <tr>
        <td id="L2572" class="blob-num js-line-number" data-line-number="2572"></td>
        <td id="LC2572" class="blob-code blob-code-inner js-file-line">								pLink-&gt;m_LinkMOEAry[(<span class="pl-k">int</span>)(CurrentTime)].<span class="pl-smi">TrafficStateCode</span> = <span class="pl-c1">2</span>;  <span class="pl-c">// 2: fully congested</span></td>
      </tr>
      <tr>
        <td id="L2573" class="blob-num js-line-number" data-line-number="2573"></td>
        <td id="LC2573" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2574" class="blob-num js-line-number" data-line-number="2574"></td>
        <td id="LC2574" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">//							TRACE(&quot;Queue spillback at %d -&gt; %d\n&quot;, g_NodeVector[pLink-&gt;m_FromNodeID], g_NodeVector[pLink-&gt;m_ToNodeID]);</span></td>
      </tr>
      <tr>
        <td id="L2575" class="blob-num js-line-number" data-line-number="2575"></td>
        <td id="LC2575" class="blob-code blob-code-inner js-file-line">							}</td>
      </tr>
      <tr>
        <td id="L2576" class="blob-num js-line-number" data-line-number="2576"></td>
        <td id="LC2576" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L2577" class="blob-num js-line-number" data-line-number="2577"></td>
        <td id="LC2577" class="blob-code blob-code-inner js-file-line">					}</td>
      </tr>
      <tr>
        <td id="L2578" class="blob-num js-line-number" data-line-number="2578"></td>
        <td id="LC2578" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L2579" class="blob-num js-line-number" data-line-number="2579"></td>
        <td id="LC2579" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2580" class="blob-num js-line-number" data-line-number="2580"></td>
        <td id="LC2580" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2581" class="blob-num js-line-number" data-line-number="2581"></td>
        <td id="LC2581" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">float</span> InflowRate = MaximumFlowRate *g_MinimumInFlowRatio;</td>
      </tr>
      <tr>
        <td id="L2582" class="blob-num js-line-number" data-line-number="2582"></td>
        <td id="LC2582" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2583" class="blob-num js-line-number" data-line-number="2583"></td>
        <td id="LC2583" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (<span class="pl-smi">fLinkInCapacity</span> &lt; InflowRate)  <span class="pl-c">// minimum inflow capacity to keep the network flowing</span></td>
      </tr>
      <tr>
        <td id="L2584" class="blob-num js-line-number" data-line-number="2584"></td>
        <td id="LC2584" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L2585" class="blob-num js-line-number" data-line-number="2585"></td>
        <td id="LC2585" class="blob-code blob-code-inner js-file-line">					<span class="pl-smi">fLinkInCapacity</span> = InflowRate;</td>
      </tr>
      <tr>
        <td id="L2586" class="blob-num js-line-number" data-line-number="2586"></td>
        <td id="LC2586" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L2587" class="blob-num js-line-number" data-line-number="2587"></td>
        <td id="LC2587" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2588" class="blob-num js-line-number" data-line-number="2588"></td>
        <td id="LC2588" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L2589" class="blob-num js-line-number" data-line-number="2589"></td>
        <td id="LC2589" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2590" class="blob-num js-line-number" data-line-number="2590"></td>
        <td id="LC2590" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">// finally we convert the floating-point capacity to integral capacity in terms of number of vehicles</span></td>
      </tr>
      <tr>
        <td id="L2591" class="blob-num js-line-number" data-line-number="2591"></td>
        <td id="LC2591" class="blob-code blob-code-inner js-file-line">			pLink-&gt;LinkInCapacity = <span class="pl-c1">g_GetRandomInteger_From_FloatingPointValue_BasedOnLinkIDAndTimeStamp</span>(<span class="pl-smi">fLinkInCapacity</span>, li);</td>
      </tr>
      <tr>
        <td id="L2592" class="blob-num js-line-number" data-line-number="2592"></td>
        <td id="LC2592" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2593" class="blob-num js-line-number" data-line-number="2593"></td>
        <td id="LC2593" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (debug_flag &amp;&amp; link_id_trace == li)</td>
      </tr>
      <tr>
        <td id="L2594" class="blob-num js-line-number" data-line-number="2594"></td>
        <td id="LC2594" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L2595" class="blob-num js-line-number" data-line-number="2595"></td>
        <td id="LC2595" class="blob-code blob-code-inner js-file-line">				<span class="pl-c1">TRACE</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Step 3: Time <span class="pl-c1">%f</span>, Link: <span class="pl-c1">%d</span> -&gt; <span class="pl-c1">%d</span>: Incapacity <span class="pl-c1">%d</span>, <span class="pl-c1">%f</span>, OutCapacity: <span class="pl-c1">%d</span><span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>, CurrentTime, g_NodeVector[pLink-&gt;m_FromNodeID].<span class="pl-smi">m_NodeNumber</span>, g_NodeVector[pLink-&gt;m_ToNodeID].<span class="pl-smi">m_NodeNumber</span>, pLink-&gt;LinkInCapacity, <span class="pl-smi">fLinkInCapacity</span>, pLink-&gt;LinkOutCapacity);</td>
      </tr>
      <tr>
        <td id="L2596" class="blob-num js-line-number" data-line-number="2596"></td>
        <td id="LC2596" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L2597" class="blob-num js-line-number" data-line-number="2597"></td>
        <td id="LC2597" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L2598" class="blob-num js-line-number" data-line-number="2598"></td>
        <td id="LC2598" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2599" class="blob-num js-line-number" data-line-number="2599"></td>
        <td id="LC2599" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2600" class="blob-num js-line-number" data-line-number="2600"></td>
        <td id="LC2600" class="blob-code blob-code-inner js-file-line">		<span class="pl-c">// distribute link in capacity to different incoming links</span></td>
      </tr>
      <tr>
        <td id="L2601" class="blob-num js-line-number" data-line-number="2601"></td>
        <td id="LC2601" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">for</span> (<span class="pl-k">unsigned</span> li = <span class="pl-c1">0</span>; li&lt; link_size; li++)</td>
      </tr>
      <tr>
        <td id="L2602" class="blob-num js-line-number" data-line-number="2602"></td>
        <td id="LC2602" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L2603" class="blob-num js-line-number" data-line-number="2603"></td>
        <td id="LC2603" class="blob-code blob-code-inner js-file-line">			DTALink* pLink = g_LinkVector[li];</td>
      </tr>
      <tr>
        <td id="L2604" class="blob-num js-line-number" data-line-number="2604"></td>
        <td id="LC2604" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">unsigned</span> <span class="pl-k">int</span> il;</td>
      </tr>
      <tr>
        <td id="L2605" class="blob-num js-line-number" data-line-number="2605"></td>
        <td id="LC2605" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (pLink-&gt;m_bMergeFlag &gt;= <span class="pl-c1">1</span>)</td>
      </tr>
      <tr>
        <td id="L2606" class="blob-num js-line-number" data-line-number="2606"></td>
        <td id="LC2606" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L2607" class="blob-num js-line-number" data-line-number="2607"></td>
        <td id="LC2607" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">int</span> TotalInFlowCount = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L2608" class="blob-num js-line-number" data-line-number="2608"></td>
        <td id="LC2608" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">for</span> (il = <span class="pl-c1">0</span>; il&lt; pLink-&gt;MergeIncomingLinkVector.<span class="pl-c1">size</span>(); il++)</td>
      </tr>
      <tr>
        <td id="L2609" class="blob-num js-line-number" data-line-number="2609"></td>
        <td id="LC2609" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L2610" class="blob-num js-line-number" data-line-number="2610"></td>
        <td id="LC2610" class="blob-code blob-code-inner js-file-line">					TotalInFlowCount += g_LinkVector[pLink-&gt;MergeIncomingLinkVector[il].<span class="pl-smi">m_LinkNo</span>]-&gt;ExitQueue.<span class="pl-c1">size</span>();  <span class="pl-c">// count vehiciles waiting at exit queue</span></td>
      </tr>
      <tr>
        <td id="L2611" class="blob-num js-line-number" data-line-number="2611"></td>
        <td id="LC2611" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2612" class="blob-num js-line-number" data-line-number="2612"></td>
        <td id="LC2612" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L2613" class="blob-num js-line-number" data-line-number="2613"></td>
        <td id="LC2613" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2614" class="blob-num js-line-number" data-line-number="2614"></td>
        <td id="LC2614" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (TotalInFlowCount &gt; pLink-&gt;LinkInCapacity)  <span class="pl-c">// demand &gt; supply</span></td>
      </tr>
      <tr>
        <td id="L2615" class="blob-num js-line-number" data-line-number="2615"></td>
        <td id="LC2615" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L2616" class="blob-num js-line-number" data-line-number="2616"></td>
        <td id="LC2616" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2617" class="blob-num js-line-number" data-line-number="2617"></td>
        <td id="LC2617" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">if</span> (pLink-&gt;m_bMergeFlag == <span class="pl-c1">1</span>)  <span class="pl-c">// merge with mulitple freeway/onramp links only</span></td>
      </tr>
      <tr>
        <td id="L2618" class="blob-num js-line-number" data-line-number="2618"></td>
        <td id="LC2618" class="blob-code blob-code-inner js-file-line">					{</td>
      </tr>
      <tr>
        <td id="L2619" class="blob-num js-line-number" data-line-number="2619"></td>
        <td id="LC2619" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2620" class="blob-num js-line-number" data-line-number="2620"></td>
        <td id="LC2620" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">if</span> (g_MergeNodeModelFlag == <span class="pl-c1">0</span>)</td>
      </tr>
      <tr>
        <td id="L2621" class="blob-num js-line-number" data-line-number="2621"></td>
        <td id="LC2621" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L2622" class="blob-num js-line-number" data-line-number="2622"></td>
        <td id="LC2622" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">// distribute capacity according to number of lanes, defined previously.</span></td>
      </tr>
      <tr>
        <td id="L2623" class="blob-num js-line-number" data-line-number="2623"></td>
        <td id="LC2623" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L2624" class="blob-num js-line-number" data-line-number="2624"></td>
        <td id="LC2624" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2625" class="blob-num js-line-number" data-line-number="2625"></td>
        <td id="LC2625" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">if</span> (g_MergeNodeModelFlag == <span class="pl-c1">1</span>)</td>
      </tr>
      <tr>
        <td id="L2626" class="blob-num js-line-number" data-line-number="2626"></td>
        <td id="LC2626" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">// distribute capacity according to number of incoming flow waiting on the queue, but it has bias toward on ramp, as the on ramp is using piont queue model now</span></td>
      </tr>
      <tr>
        <td id="L2627" class="blob-num js-line-number" data-line-number="2627"></td>
        <td id="LC2627" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L2628" class="blob-num js-line-number" data-line-number="2628"></td>
        <td id="LC2628" class="blob-code blob-code-inner js-file-line">							<span class="pl-k">for</span> (il = <span class="pl-c1">0</span>; il&lt; pLink-&gt;MergeIncomingLinkVector.<span class="pl-c1">size</span>(); il++)</td>
      </tr>
      <tr>
        <td id="L2629" class="blob-num js-line-number" data-line-number="2629"></td>
        <td id="LC2629" class="blob-code blob-code-inner js-file-line">							{</td>
      </tr>
      <tr>
        <td id="L2630" class="blob-num js-line-number" data-line-number="2630"></td>
        <td id="LC2630" class="blob-code blob-code-inner js-file-line">								pLink-&gt;MergeIncomingLinkVector[il].<span class="pl-smi">m_LinkInCapacityRatio</span> = g_LinkVector[pLink-&gt;MergeIncomingLinkVector[il].<span class="pl-smi">m_LinkNo</span>]-&gt;ExitQueue.<span class="pl-c1">size</span>()*<span class="pl-c1">1</span>.<span class="pl-c1">0f</span> / TotalInFlowCount;</td>
      </tr>
      <tr>
        <td id="L2631" class="blob-num js-line-number" data-line-number="2631"></td>
        <td id="LC2631" class="blob-code blob-code-inner js-file-line">							}</td>
      </tr>
      <tr>
        <td id="L2632" class="blob-num js-line-number" data-line-number="2632"></td>
        <td id="LC2632" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L2633" class="blob-num js-line-number" data-line-number="2633"></td>
        <td id="LC2633" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2634" class="blob-num js-line-number" data-line-number="2634"></td>
        <td id="LC2634" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">for</span> (il = <span class="pl-c1">0</span>; il&lt; pLink-&gt;MergeIncomingLinkVector.<span class="pl-c1">size</span>(); il++)</td>
      </tr>
      <tr>
        <td id="L2635" class="blob-num js-line-number" data-line-number="2635"></td>
        <td id="LC2635" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L2636" class="blob-num js-line-number" data-line-number="2636"></td>
        <td id="LC2636" class="blob-code blob-code-inner js-file-line">							<span class="pl-k">float</span> LinkOutCapacity = pLink-&gt;LinkInCapacity * pLink-&gt;MergeIncomingLinkVector[il].<span class="pl-smi">m_LinkInCapacityRatio</span>;</td>
      </tr>
      <tr>
        <td id="L2637" class="blob-num js-line-number" data-line-number="2637"></td>
        <td id="LC2637" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2638" class="blob-num js-line-number" data-line-number="2638"></td>
        <td id="LC2638" class="blob-code blob-code-inner js-file-line">							<span class="pl-k">int</span> LinkOutCapacity_int = <span class="pl-c1">g_GetRandomInteger_From_FloatingPointValue_BasedOnLinkIDAndTimeStamp</span>(LinkOutCapacity, li);</td>
      </tr>
      <tr>
        <td id="L2639" class="blob-num js-line-number" data-line-number="2639"></td>
        <td id="LC2639" class="blob-code blob-code-inner js-file-line">							g_LinkVector[pLink-&gt;MergeIncomingLinkVector[il].<span class="pl-smi">m_LinkNo</span>]-&gt;LinkOutCapacity = LinkOutCapacity_int;</td>
      </tr>
      <tr>
        <td id="L2640" class="blob-num js-line-number" data-line-number="2640"></td>
        <td id="LC2640" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2641" class="blob-num js-line-number" data-line-number="2641"></td>
        <td id="LC2641" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L2642" class="blob-num js-line-number" data-line-number="2642"></td>
        <td id="LC2642" class="blob-code blob-code-inner js-file-line">					}</td>
      </tr>
      <tr>
        <td id="L2643" class="blob-num js-line-number" data-line-number="2643"></td>
        <td id="LC2643" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">if</span> (pLink-&gt;m_bMergeFlag == <span class="pl-c1">2</span>)  <span class="pl-c">// merge with onramp</span></td>
      </tr>
      <tr>
        <td id="L2644" class="blob-num js-line-number" data-line-number="2644"></td>
        <td id="LC2644" class="blob-code blob-code-inner js-file-line">					{</td>
      </tr>
      <tr>
        <td id="L2645" class="blob-num js-line-number" data-line-number="2645"></td>
        <td id="LC2645" class="blob-code blob-code-inner js-file-line">						<span class="pl-c">// step a. check with flow on onramp</span></td>
      </tr>
      <tr>
        <td id="L2646" class="blob-num js-line-number" data-line-number="2646"></td>
        <td id="LC2646" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">float</span> MaxMergeCapacity = g_LinkVector[pLink-&gt;m_MergeOnrampLinkID]-&gt;<span class="pl-c1">GetHourlyPerLaneCapacity</span>(CurrentTime)*g_DTASimulationInterval / <span class="pl-c1">60</span>.<span class="pl-c1">0f</span>*g_LinkVector[pLink-&gt;m_MergeOnrampLinkID]-&gt;<span class="pl-c1">GetNumberOfLanes</span>(DayNo, CurrentTime) * <span class="pl-c1">0</span>.<span class="pl-c1">5f</span>; <span class="pl-c">//60 --&gt; cap per min --&gt; unit # of vehicle per simulation interval</span></td>
      </tr>
      <tr>
        <td id="L2647" class="blob-num js-line-number" data-line-number="2647"></td>
        <td id="LC2647" class="blob-code blob-code-inner js-file-line">						<span class="pl-c">// 0.5f -&gt; half of the onramp capacity</span></td>
      </tr>
      <tr>
        <td id="L2648" class="blob-num js-line-number" data-line-number="2648"></td>
        <td id="LC2648" class="blob-code blob-code-inner js-file-line">						<span class="pl-c">// use integer number of vehicles as unit of capacity</span></td>
      </tr>
      <tr>
        <td id="L2649" class="blob-num js-line-number" data-line-number="2649"></td>
        <td id="LC2649" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2650" class="blob-num js-line-number" data-line-number="2650"></td>
        <td id="LC2650" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">unsigned</span> <span class="pl-k">int</span> MaxMergeCapacity_int = <span class="pl-c1">g_GetRandomInteger_From_FloatingPointValue_BasedOnLinkIDAndTimeStamp</span>(MaxMergeCapacity, li);</td>
      </tr>
      <tr>
        <td id="L2651" class="blob-num js-line-number" data-line-number="2651"></td>
        <td id="LC2651" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2652" class="blob-num js-line-number" data-line-number="2652"></td>
        <td id="LC2652" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">unsigned</span> <span class="pl-k">int</span> FlowonOnRamp = g_LinkVector[pLink-&gt;m_MergeOnrampLinkID]-&gt;ExitQueue.<span class="pl-c1">size</span>();</td>
      </tr>
      <tr>
        <td id="L2653" class="blob-num js-line-number" data-line-number="2653"></td>
        <td id="LC2653" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">int</span> DownstreamLinkInCapacity = pLink-&gt;LinkInCapacity;</td>
      </tr>
      <tr>
        <td id="L2654" class="blob-num js-line-number" data-line-number="2654"></td>
        <td id="LC2654" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2655" class="blob-num js-line-number" data-line-number="2655"></td>
        <td id="LC2655" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">if</span> (FlowonOnRamp &gt; MaxMergeCapacity_int)</td>
      </tr>
      <tr>
        <td id="L2656" class="blob-num js-line-number" data-line-number="2656"></td>
        <td id="LC2656" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">// ramp flow &gt; max merge capacity on ramp  </span></td>
      </tr>
      <tr>
        <td id="L2657" class="blob-num js-line-number" data-line-number="2657"></td>
        <td id="LC2657" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">// over regions I and II in Dr. Rouphail&#39;s diagram</span></td>
      </tr>
      <tr>
        <td id="L2658" class="blob-num js-line-number" data-line-number="2658"></td>
        <td id="LC2658" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">// capacity on ramp = max merge capacity on ramp</span></td>
      </tr>
      <tr>
        <td id="L2659" class="blob-num js-line-number" data-line-number="2659"></td>
        <td id="LC2659" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">// capacity on main line = LinkInCapacity - capacity on ramp</span></td>
      </tr>
      <tr>
        <td id="L2660" class="blob-num js-line-number" data-line-number="2660"></td>
        <td id="LC2660" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">// if flow on main line &gt; capacity on main line  // queue on main line</span></td>
      </tr>
      <tr>
        <td id="L2661" class="blob-num js-line-number" data-line-number="2661"></td>
        <td id="LC2661" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">// elsewise, no queue on mainline</span></td>
      </tr>
      <tr>
        <td id="L2662" class="blob-num js-line-number" data-line-number="2662"></td>
        <td id="LC2662" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L2663" class="blob-num js-line-number" data-line-number="2663"></td>
        <td id="LC2663" class="blob-code blob-code-inner js-file-line">							g_LinkVector[pLink-&gt;m_MergeOnrampLinkID]-&gt;LinkOutCapacity = MaxMergeCapacity_int;</td>
      </tr>
      <tr>
        <td id="L2664" class="blob-num js-line-number" data-line-number="2664"></td>
        <td id="LC2664" class="blob-code blob-code-inner js-file-line">							g_LinkVector[pLink-&gt;m_MergeMainlineLinkID]-&gt;LinkOutCapacity = DownstreamLinkInCapacity - MaxMergeCapacity_int;</td>
      </tr>
      <tr>
        <td id="L2665" class="blob-num js-line-number" data-line-number="2665"></td>
        <td id="LC2665" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2666" class="blob-num js-line-number" data-line-number="2666"></td>
        <td id="LC2666" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L2667" class="blob-num js-line-number" data-line-number="2667"></td>
        <td id="LC2667" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">else</span> <span class="pl-c">// ramp flow &lt;= max merge capacity on ramp  // region III  </span></td>
      </tr>
      <tr>
        <td id="L2668" class="blob-num js-line-number" data-line-number="2668"></td>
        <td id="LC2668" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">// restrict the mainly capacity  // mainly capacity = LinkInCapacity - flow on ramp</span></td>
      </tr>
      <tr>
        <td id="L2669" class="blob-num js-line-number" data-line-number="2669"></td>
        <td id="LC2669" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L2670" class="blob-num js-line-number" data-line-number="2670"></td>
        <td id="LC2670" class="blob-code blob-code-inner js-file-line">							g_LinkVector[pLink-&gt;m_MergeOnrampLinkID]-&gt;LinkOutCapacity = MaxMergeCapacity_int;</td>
      </tr>
      <tr>
        <td id="L2671" class="blob-num js-line-number" data-line-number="2671"></td>
        <td id="LC2671" class="blob-code blob-code-inner js-file-line">							g_LinkVector[pLink-&gt;m_MergeMainlineLinkID]-&gt;LinkOutCapacity = DownstreamLinkInCapacity - FlowonOnRamp;</td>
      </tr>
      <tr>
        <td id="L2672" class="blob-num js-line-number" data-line-number="2672"></td>
        <td id="LC2672" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2673" class="blob-num js-line-number" data-line-number="2673"></td>
        <td id="LC2673" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L2674" class="blob-num js-line-number" data-line-number="2674"></td>
        <td id="LC2674" class="blob-code blob-code-inner js-file-line">						<span class="pl-c">//		g_LogFile &lt;&lt; &quot;merge: mainline capacity&quot;&lt;&lt; CurrentTime &lt;&lt; &quot; &quot;  &lt;&lt; g_LinkVector [pLink-&gt;m_MergeMainlineLinkID] -&gt;LinkOutCapacity &lt;&lt; endl;</span></td>
      </tr>
      <tr>
        <td id="L2675" class="blob-num js-line-number" data-line-number="2675"></td>
        <td id="LC2675" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2676" class="blob-num js-line-number" data-line-number="2676"></td>
        <td id="LC2676" class="blob-code blob-code-inner js-file-line">					}</td>
      </tr>
      <tr>
        <td id="L2677" class="blob-num js-line-number" data-line-number="2677"></td>
        <td id="LC2677" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L2678" class="blob-num js-line-number" data-line-number="2678"></td>
        <td id="LC2678" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L2679" class="blob-num js-line-number" data-line-number="2679"></td>
        <td id="LC2679" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L2680" class="blob-num js-line-number" data-line-number="2680"></td>
        <td id="LC2680" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L2681" class="blob-num js-line-number" data-line-number="2681"></td>
        <td id="LC2681" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2682" class="blob-num js-line-number" data-line-number="2682"></td>
        <td id="LC2682" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2683" class="blob-num js-line-number" data-line-number="2683"></td>
        <td id="LC2683" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">// step 4: move vehicles from ExitQueue to next link&#39;s EntranceQueue, if there is available capacity</span></td>
      </tr>
      <tr>
        <td id="L2684" class="blob-num js-line-number" data-line-number="2684"></td>
        <td id="LC2684" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">// NewTime = ReadyTime + FFTT(next link)</span></td>
      </tr>
      <tr>
        <td id="L2685" class="blob-num js-line-number" data-line-number="2685"></td>
        <td id="LC2685" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2686" class="blob-num js-line-number" data-line-number="2686"></td>
        <td id="LC2686" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2687" class="blob-num js-line-number" data-line-number="2687"></td>
        <td id="LC2687" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">// step 4.1: calculate movement capacity per simulation interval for movements defined in input_movement.csv</span></td>
      </tr>
      <tr>
        <td id="L2688" class="blob-num js-line-number" data-line-number="2688"></td>
        <td id="LC2688" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">// we will not parallel computing mode for this movement capacity now</span></td>
      </tr>
      <tr>
        <td id="L2689" class="blob-num js-line-number" data-line-number="2689"></td>
        <td id="LC2689" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> node_size = g_NodeVector.<span class="pl-c1">size</span>();</td>
      </tr>
      <tr>
        <td id="L2690" class="blob-num js-line-number" data-line-number="2690"></td>
        <td id="LC2690" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">for</span> (<span class="pl-k">unsigned</span> node = <span class="pl-c1">0</span>; node &lt; node_size; node++)</td>
      </tr>
      <tr>
        <td id="L2691" class="blob-num js-line-number" data-line-number="2691"></td>
        <td id="LC2691" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L2692" class="blob-num js-line-number" data-line-number="2692"></td>
        <td id="LC2692" class="blob-code blob-code-inner js-file-line">		DTANode* pNode = &amp;(g_NodeVector[node]);</td>
      </tr>
      <tr>
        <td id="L2693" class="blob-num js-line-number" data-line-number="2693"></td>
        <td id="LC2693" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">for</span> (std::map&lt;string, DTANodeMovement&gt;::iterator iter = pNode-&gt;m_MovementMap.<span class="pl-c1">begin</span>();</td>
      </tr>
      <tr>
        <td id="L2694" class="blob-num js-line-number" data-line-number="2694"></td>
        <td id="LC2694" class="blob-code blob-code-inner js-file-line">			iter != pNode-&gt;m_MovementMap.<span class="pl-c1">end</span>(); iter++)</td>
      </tr>
      <tr>
        <td id="L2695" class="blob-num js-line-number" data-line-number="2695"></td>
        <td id="LC2695" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L2696" class="blob-num js-line-number" data-line-number="2696"></td>
        <td id="LC2696" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (iter-&gt;second.<span class="pl-smi">movement_hourly_capacity</span> &gt;= <span class="pl-c1">0</span>)  <span class="pl-c">// if data are available</span></td>
      </tr>
      <tr>
        <td id="L2697" class="blob-num js-line-number" data-line-number="2697"></td>
        <td id="LC2697" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L2698" class="blob-num js-line-number" data-line-number="2698"></td>
        <td id="LC2698" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2699" class="blob-num js-line-number" data-line-number="2699"></td>
        <td id="LC2699" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">float</span> movement_hourly_capacity = iter-&gt;second.<span class="pl-smi">movement_hourly_capacity</span> / <span class="pl-c1">60</span> * g_DTASimulationInterval;</td>
      </tr>
      <tr>
        <td id="L2700" class="blob-num js-line-number" data-line-number="2700"></td>
        <td id="LC2700" class="blob-code blob-code-inner js-file-line">				iter-&gt;second.<span class="pl-smi">movement_capacity_per_simulation_interval</span> = <span class="pl-c1">g_GetRandomInteger_SingleProcessorMode</span>(movement_hourly_capacity); <span class="pl-c">// hourly -&gt; min -&gt; 6 seconds);</span></td>
      </tr>
      <tr>
        <td id="L2701" class="blob-num js-line-number" data-line-number="2701"></td>
        <td id="LC2701" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2702" class="blob-num js-line-number" data-line-number="2702"></td>
        <td id="LC2702" class="blob-code blob-code-inner js-file-line">				iter-&gt;second.<span class="pl-smi">movement_vehicle_counter</span> = <span class="pl-c1">0</span>; <span class="pl-c">// reset counter of passing vehicles to zero for each simulation interval</span></td>
      </tr>
      <tr>
        <td id="L2703" class="blob-num js-line-number" data-line-number="2703"></td>
        <td id="LC2703" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2704" class="blob-num js-line-number" data-line-number="2704"></td>
        <td id="LC2704" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">//		TRACE(&quot;\n hourly cap: %f, cap per simulation interval %d&quot;,movement_hourly_capacity, iter-&gt;second.movement_capacity_per_simulation_interval);</span></td>
      </tr>
      <tr>
        <td id="L2705" class="blob-num js-line-number" data-line-number="2705"></td>
        <td id="LC2705" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2706" class="blob-num js-line-number" data-line-number="2706"></td>
        <td id="LC2706" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L2707" class="blob-num js-line-number" data-line-number="2707"></td>
        <td id="LC2707" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2708" class="blob-num js-line-number" data-line-number="2708"></td>
        <td id="LC2708" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2709" class="blob-num js-line-number" data-line-number="2709"></td>
        <td id="LC2709" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L2710" class="blob-num js-line-number" data-line-number="2710"></td>
        <td id="LC2710" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2711" class="blob-num js-line-number" data-line-number="2711"></td>
        <td id="LC2711" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L2712" class="blob-num js-line-number" data-line-number="2712"></td>
        <td id="LC2712" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2713" class="blob-num js-line-number" data-line-number="2713"></td>
        <td id="LC2713" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2714" class="blob-num js-line-number" data-line-number="2714"></td>
        <td id="LC2714" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">int</span> NextLink;</td>
      </tr>
      <tr>
        <td id="L2715" class="blob-num js-line-number" data-line-number="2715"></td>
        <td id="LC2715" class="blob-code blob-code-inner js-file-line">	DTALink* p_Nextlink;</td>
      </tr>
      <tr>
        <td id="L2716" class="blob-num js-line-number" data-line-number="2716"></td>
        <td id="LC2716" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2717" class="blob-num js-line-number" data-line-number="2717"></td>
        <td id="LC2717" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//for each node, we scan each incoming link in a randomly sequence, based on meso_simulation_time_interval_no</span></td>
      </tr>
      <tr>
        <td id="L2718" class="blob-num js-line-number" data-line-number="2718"></td>
        <td id="LC2718" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2719" class="blob-num js-line-number" data-line-number="2719"></td>
        <td id="LC2719" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">pragma</span> omp parallel for</td>
      </tr>
      <tr>
        <td id="L2720" class="blob-num js-line-number" data-line-number="2720"></td>
        <td id="LC2720" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">for</span> (<span class="pl-k">int</span> node = <span class="pl-c1">0</span>; node &lt; node_size; node++)</td>
      </tr>
      <tr>
        <td id="L2721" class="blob-num js-line-number" data-line-number="2721"></td>
        <td id="LC2721" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L2722" class="blob-num js-line-number" data-line-number="2722"></td>
        <td id="LC2722" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">int</span> IncomingLinkSize = g_NodeVector[node].<span class="pl-smi">m_IncomingLinkVector</span>.<span class="pl-c1">size</span>();</td>
      </tr>
      <tr>
        <td id="L2723" class="blob-num js-line-number" data-line-number="2723"></td>
        <td id="LC2723" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">int</span> incoming_link_count = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L2724" class="blob-num js-line-number" data-line-number="2724"></td>
        <td id="LC2724" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2725" class="blob-num js-line-number" data-line-number="2725"></td>
        <td id="LC2725" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">int</span> incoming_link_sequence = meso_simulation_time_interval_no%<span class="pl-c1">max</span>(<span class="pl-c1">1</span>, IncomingLinkSize);  <span class="pl-c">// random start point</span></td>
      </tr>
      <tr>
        <td id="L2726" class="blob-num js-line-number" data-line-number="2726"></td>
        <td id="LC2726" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2727" class="blob-num js-line-number" data-line-number="2727"></td>
        <td id="LC2727" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">while</span> (incoming_link_count &lt; IncomingLinkSize)</td>
      </tr>
      <tr>
        <td id="L2728" class="blob-num js-line-number" data-line-number="2728"></td>
        <td id="LC2728" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L2729" class="blob-num js-line-number" data-line-number="2729"></td>
        <td id="LC2729" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2730" class="blob-num js-line-number" data-line-number="2730"></td>
        <td id="LC2730" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">int</span> li = g_NodeVector[node].<span class="pl-smi">m_IncomingLinkVector</span>[incoming_link_sequence];</td>
      </tr>
      <tr>
        <td id="L2731" class="blob-num js-line-number" data-line-number="2731"></td>
        <td id="LC2731" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2732" class="blob-num js-line-number" data-line-number="2732"></td>
        <td id="LC2732" class="blob-code blob-code-inner js-file-line">			DTALink* pLink = g_LinkVector[li];</td>
      </tr>
      <tr>
        <td id="L2733" class="blob-num js-line-number" data-line-number="2733"></td>
        <td id="LC2733" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">//if(debug_flag &amp;&amp; (pLink-&gt;m_FromNodeNumber  == 10 &amp;&amp; pLink-&gt;m_ToNodeNumber == 5) &amp;&amp; CurrentTime &gt;=480)</span></td>
      </tr>
      <tr>
        <td id="L2734" class="blob-num js-line-number" data-line-number="2734"></td>
        <td id="LC2734" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">//{</span></td>
      </tr>
      <tr>
        <td id="L2735" class="blob-num js-line-number" data-line-number="2735"></td>
        <td id="LC2735" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">//	TRACE(&quot;Step 3: Time %f, Link: %d -&gt; %d: tracing \n&quot;, CurrentTime, g_NodeVector[pLink-&gt;m_FromNodeID].m_NodeNumber , g_NodeVector[pLink-&gt;m_ToNodeID].m_NodeNumber);</span></td>
      </tr>
      <tr>
        <td id="L2736" class="blob-num js-line-number" data-line-number="2736"></td>
        <td id="LC2736" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">//}</span></td>
      </tr>
      <tr>
        <td id="L2737" class="blob-num js-line-number" data-line-number="2737"></td>
        <td id="LC2737" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2738" class="blob-num js-line-number" data-line-number="2738"></td>
        <td id="LC2738" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">// vehicle_out_count is the minimum of LinkOutCapacity and ExitQueue Size</span></td>
      </tr>
      <tr>
        <td id="L2739" class="blob-num js-line-number" data-line-number="2739"></td>
        <td id="LC2739" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2740" class="blob-num js-line-number" data-line-number="2740"></td>
        <td id="LC2740" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">int</span> vehicle_out_count = pLink-&gt;LinkOutCapacity;</td>
      </tr>
      <tr>
        <td id="L2741" class="blob-num js-line-number" data-line-number="2741"></td>
        <td id="LC2741" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2742" class="blob-num js-line-number" data-line-number="2742"></td>
        <td id="LC2742" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">//			g_LogFile &lt;&lt; &quot;link out capaity:&quot;&lt;&lt; CurrentTime &lt;&lt; &quot; &quot;  &lt;&lt; g_NodeVector[pLink-&gt;m_FromNodeID] &lt;&lt; &quot; -&gt;&quot; &lt;&lt; g_NodeVector[pLink-&gt;m_ToNodeID]&lt;&lt;&quot; Cap:&quot; &lt;&lt; vehicle_out_count&lt;&lt; &quot;queue:&quot; &lt;&lt; pLink-&gt;ExitQueue.size() &lt;&lt; endl;</span></td>
      </tr>
      <tr>
        <td id="L2743" class="blob-num js-line-number" data-line-number="2743"></td>
        <td id="LC2743" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2744" class="blob-num js-line-number" data-line-number="2744"></td>
        <td id="LC2744" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (pLink-&gt;ExitQueue.<span class="pl-c1">size</span>() &lt;= pLink-&gt;LinkOutCapacity)</td>
      </tr>
      <tr>
        <td id="L2745" class="blob-num js-line-number" data-line-number="2745"></td>
        <td id="LC2745" class="blob-code blob-code-inner js-file-line">			{      <span class="pl-c">// under capacity, constrained by existing queue</span></td>
      </tr>
      <tr>
        <td id="L2746" class="blob-num js-line-number" data-line-number="2746"></td>
        <td id="LC2746" class="blob-code blob-code-inner js-file-line">				vehicle_out_count = pLink-&gt;ExitQueue.<span class="pl-c1">size</span>();</td>
      </tr>
      <tr>
        <td id="L2747" class="blob-num js-line-number" data-line-number="2747"></td>
        <td id="LC2747" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L2748" class="blob-num js-line-number" data-line-number="2748"></td>
        <td id="LC2748" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2749" class="blob-num js-line-number" data-line-number="2749"></td>
        <td id="LC2749" class="blob-code blob-code-inner js-file-line">			list&lt;struc_vehicle_item&gt;::iterator exit_queue_it = pLink-&gt;ExitQueue.<span class="pl-c1">begin</span>();</td>
      </tr>
      <tr>
        <td id="L2750" class="blob-num js-line-number" data-line-number="2750"></td>
        <td id="LC2750" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2751" class="blob-num js-line-number" data-line-number="2751"></td>
        <td id="LC2751" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">while</span> (pLink-&gt;ExitQueue.<span class="pl-c1">size</span>() &gt;<span class="pl-c1">0</span> &amp;&amp; vehicle_out_count &gt;<span class="pl-c1">0</span> &amp;&amp; exit_queue_it != pLink-&gt;ExitQueue.<span class="pl-c1">end</span>())  <span class="pl-c">// go through</span></td>
      </tr>
      <tr>
        <td id="L2752" class="blob-num js-line-number" data-line-number="2752"></td>
        <td id="LC2752" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L2753" class="blob-num js-line-number" data-line-number="2753"></td>
        <td id="LC2753" class="blob-code blob-code-inner js-file-line">				struc_vehicle_item vi = (*exit_queue_it);</td>
      </tr>
      <tr>
        <td id="L2754" class="blob-num js-line-number" data-line-number="2754"></td>
        <td id="LC2754" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2755" class="blob-num js-line-number" data-line-number="2755"></td>
        <td id="LC2755" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">int</span> vehicle_id = vi.<span class="pl-smi">veh_id</span>;</td>
      </tr>
      <tr>
        <td id="L2756" class="blob-num js-line-number" data-line-number="2756"></td>
        <td id="LC2756" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2757" class="blob-num js-line-number" data-line-number="2757"></td>
        <td id="LC2757" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">// record arrival time at the downstream node of current link</span></td>
      </tr>
      <tr>
        <td id="L2758" class="blob-num js-line-number" data-line-number="2758"></td>
        <td id="LC2758" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">int</span> link_sequence_no = g_VehicleMap[vehicle_id]-&gt;m_SimLinkSequenceNo;</td>
      </tr>
      <tr>
        <td id="L2759" class="blob-num js-line-number" data-line-number="2759"></td>
        <td id="LC2759" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2760" class="blob-num js-line-number" data-line-number="2760"></td>
        <td id="LC2760" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">int</span> t_link_arrival_time = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L2761" class="blob-num js-line-number" data-line-number="2761"></td>
        <td id="LC2761" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (link_sequence_no &gt;= <span class="pl-c1">1</span>)</td>
      </tr>
      <tr>
        <td id="L2762" class="blob-num js-line-number" data-line-number="2762"></td>
        <td id="LC2762" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L2763" class="blob-num js-line-number" data-line-number="2763"></td>
        <td id="LC2763" class="blob-code blob-code-inner js-file-line">					t_link_arrival_time = <span class="pl-c1">int</span>(g_VehicleMap[vehicle_id]-&gt;m_LinkAry[link_sequence_no - <span class="pl-c1">1</span>].<span class="pl-smi">AbsArrivalTimeOnDSN</span>);</td>
      </tr>
      <tr>
        <td id="L2764" class="blob-num js-line-number" data-line-number="2764"></td>
        <td id="LC2764" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L2765" class="blob-num js-line-number" data-line-number="2765"></td>
        <td id="LC2765" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L2766" class="blob-num js-line-number" data-line-number="2766"></td>
        <td id="LC2766" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L2767" class="blob-num js-line-number" data-line-number="2767"></td>
        <td id="LC2767" class="blob-code blob-code-inner js-file-line">					t_link_arrival_time = <span class="pl-c1">int</span>(g_VehicleMap[vehicle_id]-&gt;m_DepartureTime);</td>
      </tr>
      <tr>
        <td id="L2768" class="blob-num js-line-number" data-line-number="2768"></td>
        <td id="LC2768" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L2769" class="blob-num js-line-number" data-line-number="2769"></td>
        <td id="LC2769" class="blob-code blob-code-inner js-file-line">				<span class="pl-c">// not reach destination yet</span></td>
      </tr>
      <tr>
        <td id="L2770" class="blob-num js-line-number" data-line-number="2770"></td>
        <td id="LC2770" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">int</span> number_of_links = g_VehicleMap[vehicle_id]-&gt;m_NodeSize - <span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L2771" class="blob-num js-line-number" data-line-number="2771"></td>
        <td id="LC2771" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">if</span> (g_VehicleMap[vehicle_id]-&gt;m_SimLinkSequenceNo &lt; number_of_links - <span class="pl-c1">1</span>)  <span class="pl-c">// not reach destination yet</span></td>
      </tr>
      <tr>
        <td id="L2772" class="blob-num js-line-number" data-line-number="2772"></td>
        <td id="LC2772" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L2773" class="blob-num js-line-number" data-line-number="2773"></td>
        <td id="LC2773" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2774" class="blob-num js-line-number" data-line-number="2774"></td>
        <td id="LC2774" class="blob-code blob-code-inner js-file-line">					<span class="pl-c">// advance to next link</span></td>
      </tr>
      <tr>
        <td id="L2775" class="blob-num js-line-number" data-line-number="2775"></td>
        <td id="LC2775" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2776" class="blob-num js-line-number" data-line-number="2776"></td>
        <td id="LC2776" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">if</span> (vehicle_id == vehicle_id_trace)</td>
      </tr>
      <tr>
        <td id="L2777" class="blob-num js-line-number" data-line-number="2777"></td>
        <td id="LC2777" class="blob-code blob-code-inner js-file-line">						<span class="pl-c1">TRACE</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>simulation link sequence no. <span class="pl-c1">%d</span><span class="pl-pds">&quot;</span></span>, g_VehicleMap[vehicle_id]-&gt;m_SimLinkSequenceNo);</td>
      </tr>
      <tr>
        <td id="L2778" class="blob-num js-line-number" data-line-number="2778"></td>
        <td id="LC2778" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2779" class="blob-num js-line-number" data-line-number="2779"></td>
        <td id="LC2779" class="blob-code blob-code-inner js-file-line">					NextLink = g_VehicleMap[vehicle_id]-&gt;m_LinkAry[g_VehicleMap[vehicle_id]-&gt;m_SimLinkSequenceNo + <span class="pl-c1">1</span>].<span class="pl-smi">LinkNo</span>;</td>
      </tr>
      <tr>
        <td id="L2780" class="blob-num js-line-number" data-line-number="2780"></td>
        <td id="LC2780" class="blob-code blob-code-inner js-file-line">					p_Nextlink = g_LinkVector[NextLink];</td>
      </tr>
      <tr>
        <td id="L2781" class="blob-num js-line-number" data-line-number="2781"></td>
        <td id="LC2781" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2782" class="blob-num js-line-number" data-line-number="2782"></td>
        <td id="LC2782" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">if</span> (p_Nextlink == <span class="pl-c1">NULL</span>)</td>
      </tr>
      <tr>
        <td id="L2783" class="blob-num js-line-number" data-line-number="2783"></td>
        <td id="LC2783" class="blob-code blob-code-inner js-file-line">					{</td>
      </tr>
      <tr>
        <td id="L2784" class="blob-num js-line-number" data-line-number="2784"></td>
        <td id="LC2784" class="blob-code blob-code-inner js-file-line">						<span class="pl-c1">TRACE</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Error at vehicle <span class="pl-c1">%d</span>,<span class="pl-pds">&quot;</span></span>, vehicle_id);</td>
      </tr>
      <tr>
        <td id="L2785" class="blob-num js-line-number" data-line-number="2785"></td>
        <td id="LC2785" class="blob-code blob-code-inner js-file-line">						<span class="pl-c1">ASSERT</span>(p_Nextlink != <span class="pl-c1">NULL</span>);</td>
      </tr>
      <tr>
        <td id="L2786" class="blob-num js-line-number" data-line-number="2786"></td>
        <td id="LC2786" class="blob-code blob-code-inner js-file-line">					}</td>
      </tr>
      <tr>
        <td id="L2787" class="blob-num js-line-number" data-line-number="2787"></td>
        <td id="LC2787" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2788" class="blob-num js-line-number" data-line-number="2788"></td>
        <td id="LC2788" class="blob-code blob-code-inner js-file-line">					<span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L2789" class="blob-num js-line-number" data-line-number="2789"></td>
        <td id="LC2789" class="blob-code blob-code-inner js-file-line">					<span class="pl-c">// test if movement capacity available</span></td>
      </tr>
      <tr>
        <td id="L2790" class="blob-num js-line-number" data-line-number="2790"></td>
        <td id="LC2790" class="blob-code blob-code-inner js-file-line">					<span class="pl-c">//DTANodeMovement movement_element;</span></td>
      </tr>
      <tr>
        <td id="L2791" class="blob-num js-line-number" data-line-number="2791"></td>
        <td id="LC2791" class="blob-code blob-code-inner js-file-line">					<span class="pl-c">//string movement_id;</span></td>
      </tr>
      <tr>
        <td id="L2792" class="blob-num js-line-number" data-line-number="2792"></td>
        <td id="LC2792" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2793" class="blob-num js-line-number" data-line-number="2793"></td>
        <td id="LC2793" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">if</span> (g_NodeVector[node].<span class="pl-smi">m_MovementMap</span>.<span class="pl-c1">size</span>()&gt;<span class="pl-c1">0</span>)  <span class="pl-c">// check movement capacity if there is an input movement table</span></td>
      </tr>
      <tr>
        <td id="L2794" class="blob-num js-line-number" data-line-number="2794"></td>
        <td id="LC2794" class="blob-code blob-code-inner js-file-line">					{</td>
      </tr>
      <tr>
        <td id="L2795" class="blob-num js-line-number" data-line-number="2795"></td>
        <td id="LC2795" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2796" class="blob-num js-line-number" data-line-number="2796"></td>
        <td id="LC2796" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">int</span> from_node = pLink-&gt;m_FromNodeNumber;</td>
      </tr>
      <tr>
        <td id="L2797" class="blob-num js-line-number" data-line-number="2797"></td>
        <td id="LC2797" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">int</span> to_node = pLink-&gt;m_ToNodeNumber;</td>
      </tr>
      <tr>
        <td id="L2798" class="blob-num js-line-number" data-line-number="2798"></td>
        <td id="LC2798" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">int</span> dest_node = p_Nextlink-&gt;m_ToNodeNumber;</td>
      </tr>
      <tr>
        <td id="L2799" class="blob-num js-line-number" data-line-number="2799"></td>
        <td id="LC2799" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2800" class="blob-num js-line-number" data-line-number="2800"></td>
        <td id="LC2800" class="blob-code blob-code-inner js-file-line">						string movement_id = <span class="pl-c1">GetMovementStringID</span>(from_node, to_node, dest_node);</td>
      </tr>
      <tr>
        <td id="L2801" class="blob-num js-line-number" data-line-number="2801"></td>
        <td id="LC2801" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2802" class="blob-num js-line-number" data-line-number="2802"></td>
        <td id="LC2802" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">if</span> (g_NodeVector[node].<span class="pl-smi">m_MovementMap</span>.<span class="pl-c1">find</span>(movement_id) != g_NodeVector[node].<span class="pl-smi">m_MovementMap</span>.<span class="pl-c1">end</span>()) <span class="pl-c">// the capacity for this movement has been defined</span></td>
      </tr>
      <tr>
        <td id="L2803" class="blob-num js-line-number" data-line-number="2803"></td>
        <td id="LC2803" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L2804" class="blob-num js-line-number" data-line-number="2804"></td>
        <td id="LC2804" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2805" class="blob-num js-line-number" data-line-number="2805"></td>
        <td id="LC2805" class="blob-code blob-code-inner js-file-line">							DTANodeMovement movement_element = g_NodeVector[node].<span class="pl-smi">m_MovementMap</span>[movement_id];</td>
      </tr>
      <tr>
        <td id="L2806" class="blob-num js-line-number" data-line-number="2806"></td>
        <td id="LC2806" class="blob-code blob-code-inner js-file-line">							<span class="pl-k">if</span> (movement_element.<span class="pl-smi">movement_vehicle_counter</span> &gt;= movement_element.<span class="pl-smi">movement_capacity_per_simulation_interval</span>)</td>
      </tr>
      <tr>
        <td id="L2807" class="blob-num js-line-number" data-line-number="2807"></td>
        <td id="LC2807" class="blob-code blob-code-inner js-file-line">							{ <span class="pl-c">// capacity are unavailable</span></td>
      </tr>
      <tr>
        <td id="L2808" class="blob-num js-line-number" data-line-number="2808"></td>
        <td id="LC2808" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2809" class="blob-num js-line-number" data-line-number="2809"></td>
        <td id="LC2809" class="blob-code blob-code-inner js-file-line">								vehicle_out_count--;</td>
      </tr>
      <tr>
        <td id="L2810" class="blob-num js-line-number" data-line-number="2810"></td>
        <td id="LC2810" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2811" class="blob-num js-line-number" data-line-number="2811"></td>
        <td id="LC2811" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">if</span> (g_FIFOConditionAcrossDifferentMovementFlag == <span class="pl-c1">0</span>)  <span class="pl-c">// not enforcing FIFO conditions </span></td>
      </tr>
      <tr>
        <td id="L2812" class="blob-num js-line-number" data-line-number="2812"></td>
        <td id="LC2812" class="blob-code blob-code-inner js-file-line">								{</td>
      </tr>
      <tr>
        <td id="L2813" class="blob-num js-line-number" data-line-number="2813"></td>
        <td id="LC2813" class="blob-code blob-code-inner js-file-line">									++exit_queue_it; <span class="pl-c">// move to the next vehicle</span></td>
      </tr>
      <tr>
        <td id="L2814" class="blob-num js-line-number" data-line-number="2814"></td>
        <td id="LC2814" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2815" class="blob-num js-line-number" data-line-number="2815"></td>
        <td id="LC2815" class="blob-code blob-code-inner js-file-line">									<span class="pl-k">continue</span>;  <span class="pl-c">// skip the current vehicle, try the next vehicle</span></td>
      </tr>
      <tr>
        <td id="L2816" class="blob-num js-line-number" data-line-number="2816"></td>
        <td id="LC2816" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2817" class="blob-num js-line-number" data-line-number="2817"></td>
        <td id="LC2817" class="blob-code blob-code-inner js-file-line">								}</td>
      </tr>
      <tr>
        <td id="L2818" class="blob-num js-line-number" data-line-number="2818"></td>
        <td id="LC2818" class="blob-code blob-code-inner js-file-line">								<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L2819" class="blob-num js-line-number" data-line-number="2819"></td>
        <td id="LC2819" class="blob-code blob-code-inner js-file-line">								{</td>
      </tr>
      <tr>
        <td id="L2820" class="blob-num js-line-number" data-line-number="2820"></td>
        <td id="LC2820" class="blob-code blob-code-inner js-file-line">									<span class="pl-k">break</span>; <span class="pl-c">// not move any vehicles behind this vehicle</span></td>
      </tr>
      <tr>
        <td id="L2821" class="blob-num js-line-number" data-line-number="2821"></td>
        <td id="LC2821" class="blob-code blob-code-inner js-file-line">								}</td>
      </tr>
      <tr>
        <td id="L2822" class="blob-num js-line-number" data-line-number="2822"></td>
        <td id="LC2822" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2823" class="blob-num js-line-number" data-line-number="2823"></td>
        <td id="LC2823" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2824" class="blob-num js-line-number" data-line-number="2824"></td>
        <td id="LC2824" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2825" class="blob-num js-line-number" data-line-number="2825"></td>
        <td id="LC2825" class="blob-code blob-code-inner js-file-line">							}</td>
      </tr>
      <tr>
        <td id="L2826" class="blob-num js-line-number" data-line-number="2826"></td>
        <td id="LC2826" class="blob-code blob-code-inner js-file-line">							<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L2827" class="blob-num js-line-number" data-line-number="2827"></td>
        <td id="LC2827" class="blob-code blob-code-inner js-file-line">							{</td>
      </tr>
      <tr>
        <td id="L2828" class="blob-num js-line-number" data-line-number="2828"></td>
        <td id="LC2828" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">// move to the next step to check if the link in capacity is available</span></td>
      </tr>
      <tr>
        <td id="L2829" class="blob-num js-line-number" data-line-number="2829"></td>
        <td id="LC2829" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2830" class="blob-num js-line-number" data-line-number="2830"></td>
        <td id="LC2830" class="blob-code blob-code-inner js-file-line">								<span class="pl-c">//								TRACE(&quot;move to the next step to check if the link in capacity is available&quot;);</span></td>
      </tr>
      <tr>
        <td id="L2831" class="blob-num js-line-number" data-line-number="2831"></td>
        <td id="LC2831" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2832" class="blob-num js-line-number" data-line-number="2832"></td>
        <td id="LC2832" class="blob-code blob-code-inner js-file-line">							}</td>
      </tr>
      <tr>
        <td id="L2833" class="blob-num js-line-number" data-line-number="2833"></td>
        <td id="LC2833" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2834" class="blob-num js-line-number" data-line-number="2834"></td>
        <td id="LC2834" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L2835" class="blob-num js-line-number" data-line-number="2835"></td>
        <td id="LC2835" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2836" class="blob-num js-line-number" data-line-number="2836"></td>
        <td id="LC2836" class="blob-code blob-code-inner js-file-line">					}  <span class="pl-c">// end of movement checking</span></td>
      </tr>
      <tr>
        <td id="L2837" class="blob-num js-line-number" data-line-number="2837"></td>
        <td id="LC2837" class="blob-code blob-code-inner js-file-line">					<span class="pl-c">//</span></td>
      </tr>
      <tr>
        <td id="L2838" class="blob-num js-line-number" data-line-number="2838"></td>
        <td id="LC2838" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2839" class="blob-num js-line-number" data-line-number="2839"></td>
        <td id="LC2839" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">if</span> (p_Nextlink-&gt;LinkInCapacity &gt; <span class="pl-c1">0</span>) <span class="pl-c">// if there is available spatial capacity on next link, then move to next link, otherwise, stay on the current link</span></td>
      </tr>
      <tr>
        <td id="L2840" class="blob-num js-line-number" data-line-number="2840"></td>
        <td id="LC2840" class="blob-code blob-code-inner js-file-line">					{</td>
      </tr>
      <tr>
        <td id="L2841" class="blob-num js-line-number" data-line-number="2841"></td>
        <td id="LC2841" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">float</span> ArrivalTimeOnDSN = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L2842" class="blob-num js-line-number" data-line-number="2842"></td>
        <td id="LC2842" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">if</span> (<span class="pl-c1">g_floating_point_value_less_than_or_eq_comparison</span>(CurrentTime - g_DTASimulationInterval, vi.<span class="pl-smi">event_time_stamp</span>))</td>
      </tr>
      <tr>
        <td id="L2843" class="blob-num js-line-number" data-line-number="2843"></td>
        <td id="LC2843" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">// arrival at previous interval</span></td>
      </tr>
      <tr>
        <td id="L2844" class="blob-num js-line-number" data-line-number="2844"></td>
        <td id="LC2844" class="blob-code blob-code-inner js-file-line">						{  <span class="pl-c">// no delay </span></td>
      </tr>
      <tr>
        <td id="L2845" class="blob-num js-line-number" data-line-number="2845"></td>
        <td id="LC2845" class="blob-code blob-code-inner js-file-line">							ArrivalTimeOnDSN = vi.<span class="pl-smi">event_time_stamp</span>;</td>
      </tr>
      <tr>
        <td id="L2846" class="blob-num js-line-number" data-line-number="2846"></td>
        <td id="LC2846" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L2847" class="blob-num js-line-number" data-line-number="2847"></td>
        <td id="LC2847" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L2848" class="blob-num js-line-number" data-line-number="2848"></td>
        <td id="LC2848" class="blob-code blob-code-inner js-file-line">						{  <span class="pl-c">// delayed at previous time interval, discharge at CurrentTime </span></td>
      </tr>
      <tr>
        <td id="L2849" class="blob-num js-line-number" data-line-number="2849"></td>
        <td id="LC2849" class="blob-code blob-code-inner js-file-line">							ArrivalTimeOnDSN = CurrentTime;</td>
      </tr>
      <tr>
        <td id="L2850" class="blob-num js-line-number" data-line-number="2850"></td>
        <td id="LC2850" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L2851" class="blob-num js-line-number" data-line-number="2851"></td>
        <td id="LC2851" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2852" class="blob-num js-line-number" data-line-number="2852"></td>
        <td id="LC2852" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2853" class="blob-num js-line-number" data-line-number="2853"></td>
        <td id="LC2853" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">float</span> TimeOnNextLink = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L2854" class="blob-num js-line-number" data-line-number="2854"></td>
        <td id="LC2854" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2855" class="blob-num js-line-number" data-line-number="2855"></td>
        <td id="LC2855" class="blob-code blob-code-inner js-file-line">						<span class="pl-c">// update statistics for traveled link</span></td>
      </tr>
      <tr>
        <td id="L2856" class="blob-num js-line-number" data-line-number="2856"></td>
        <td id="LC2856" class="blob-code blob-code-inner js-file-line">						g_VehicleMap[vehicle_id]-&gt;m_LinkAry[link_sequence_no].<span class="pl-smi">AbsArrivalTimeOnDSN</span> = ArrivalTimeOnDSN;</td>
      </tr>
      <tr>
        <td id="L2857" class="blob-num js-line-number" data-line-number="2857"></td>
        <td id="LC2857" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">float</span> TravelTime = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L2858" class="blob-num js-line-number" data-line-number="2858"></td>
        <td id="LC2858" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2859" class="blob-num js-line-number" data-line-number="2859"></td>
        <td id="LC2859" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">if</span> (link_sequence_no &gt;= <span class="pl-c1">1</span>)</td>
      </tr>
      <tr>
        <td id="L2860" class="blob-num js-line-number" data-line-number="2860"></td>
        <td id="LC2860" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L2861" class="blob-num js-line-number" data-line-number="2861"></td>
        <td id="LC2861" class="blob-code blob-code-inner js-file-line">							TravelTime = g_VehicleMap[vehicle_id]-&gt;m_LinkAry[link_sequence_no].<span class="pl-smi">AbsArrivalTimeOnDSN</span> -</td>
      </tr>
      <tr>
        <td id="L2862" class="blob-num js-line-number" data-line-number="2862"></td>
        <td id="LC2862" class="blob-code blob-code-inner js-file-line">								g_VehicleMap[vehicle_id]-&gt;m_LinkAry[link_sequence_no - <span class="pl-c1">1</span>].<span class="pl-smi">AbsArrivalTimeOnDSN</span>;</td>
      </tr>
      <tr>
        <td id="L2863" class="blob-num js-line-number" data-line-number="2863"></td>
        <td id="LC2863" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L2864" class="blob-num js-line-number" data-line-number="2864"></td>
        <td id="LC2864" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L2865" class="blob-num js-line-number" data-line-number="2865"></td>
        <td id="LC2865" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L2866" class="blob-num js-line-number" data-line-number="2866"></td>
        <td id="LC2866" class="blob-code blob-code-inner js-file-line">							TravelTime = g_VehicleMap[vehicle_id]-&gt;m_LinkAry[link_sequence_no].<span class="pl-smi">AbsArrivalTimeOnDSN</span> -</td>
      </tr>
      <tr>
        <td id="L2867" class="blob-num js-line-number" data-line-number="2867"></td>
        <td id="LC2867" class="blob-code blob-code-inner js-file-line">								g_VehicleMap[vehicle_id]-&gt;m_DepartureTime;</td>
      </tr>
      <tr>
        <td id="L2868" class="blob-num js-line-number" data-line-number="2868"></td>
        <td id="LC2868" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2869" class="blob-num js-line-number" data-line-number="2869"></td>
        <td id="LC2869" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L2870" class="blob-num js-line-number" data-line-number="2870"></td>
        <td id="LC2870" class="blob-code blob-code-inner js-file-line">						g_VehicleMap[vehicle_id]-&gt;m_Delay += (TravelTime - pLink-&gt;m_FreeFlowTravelTime);</td>
      </tr>
      <tr>
        <td id="L2871" class="blob-num js-line-number" data-line-number="2871"></td>
        <td id="LC2871" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2872" class="blob-num js-line-number" data-line-number="2872"></td>
        <td id="LC2872" class="blob-code blob-code-inner js-file-line">						<span class="pl-c">// finally move to next link</span></td>
      </tr>
      <tr>
        <td id="L2873" class="blob-num js-line-number" data-line-number="2873"></td>
        <td id="LC2873" class="blob-code blob-code-inner js-file-line">						g_VehicleMap[vehicle_id]-&gt;m_SimLinkSequenceNo = g_VehicleMap[vehicle_id]-&gt;m_SimLinkSequenceNo + <span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L2874" class="blob-num js-line-number" data-line-number="2874"></td>
        <td id="LC2874" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2875" class="blob-num js-line-number" data-line-number="2875"></td>
        <td id="LC2875" class="blob-code blob-code-inner js-file-line">						<span class="pl-c">// access VMS information here! p_Nextlink is now the current link</span></td>
      </tr>
      <tr>
        <td id="L2876" class="blob-num js-line-number" data-line-number="2876"></td>
        <td id="LC2876" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">int</span> IS_id = p_Nextlink-&gt;<span class="pl-c1">GetInformationResponseID</span>(DayNo, CurrentTime);</td>
      </tr>
      <tr>
        <td id="L2877" class="blob-num js-line-number" data-line-number="2877"></td>
        <td id="LC2877" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">if</span> (IS_id &gt;= <span class="pl-c1">0</span>)</td>
      </tr>
      <tr>
        <td id="L2878" class="blob-num js-line-number" data-line-number="2878"></td>
        <td id="LC2878" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L2879" class="blob-num js-line-number" data-line-number="2879"></td>
        <td id="LC2879" class="blob-code blob-code-inner js-file-line">							<span class="pl-k">if</span> (g_VehicleMap[vehicle_id]-&gt;<span class="pl-c1">GetRandomRatio</span>() * <span class="pl-c1">100</span> &lt; p_Nextlink-&gt;MessageSignVector[IS_id].<span class="pl-smi">ResponsePercentage</span>)</td>
      </tr>
      <tr>
        <td id="L2880" class="blob-num js-line-number" data-line-number="2880"></td>
        <td id="LC2880" class="blob-code blob-code-inner js-file-line">							{  <span class="pl-c">// vehicle rerouting</span></td>
      </tr>
      <tr>
        <td id="L2881" class="blob-num js-line-number" data-line-number="2881"></td>
        <td id="LC2881" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2882" class="blob-num js-line-number" data-line-number="2882"></td>
        <td id="LC2882" class="blob-code blob-code-inner js-file-line">							}</td>
      </tr>
      <tr>
        <td id="L2883" class="blob-num js-line-number" data-line-number="2883"></td>
        <td id="LC2883" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2884" class="blob-num js-line-number" data-line-number="2884"></td>
        <td id="LC2884" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L2885" class="blob-num js-line-number" data-line-number="2885"></td>
        <td id="LC2885" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2886" class="blob-num js-line-number" data-line-number="2886"></td>
        <td id="LC2886" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2887" class="blob-num js-line-number" data-line-number="2887"></td>
        <td id="LC2887" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2888" class="blob-num js-line-number" data-line-number="2888"></td>
        <td id="LC2888" class="blob-code blob-code-inner js-file-line">						vi.<span class="pl-smi">veh_id</span> = vehicle_id;</td>
      </tr>
      <tr>
        <td id="L2889" class="blob-num js-line-number" data-line-number="2889"></td>
        <td id="LC2889" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2890" class="blob-num js-line-number" data-line-number="2890"></td>
        <td id="LC2890" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">float</span> FFTT = p_Nextlink-&gt;<span class="pl-c1">GetFreeMovingTravelTime</span>(TrafficFlowModelFlag, CurrentTime);</td>
      </tr>
      <tr>
        <td id="L2891" class="blob-num js-line-number" data-line-number="2891"></td>
        <td id="LC2891" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2892" class="blob-num js-line-number" data-line-number="2892"></td>
        <td id="LC2892" class="blob-code blob-code-inner js-file-line">						vi.<span class="pl-smi">event_time_stamp</span> = ArrivalTimeOnDSN + FFTT;</td>
      </tr>
      <tr>
        <td id="L2893" class="blob-num js-line-number" data-line-number="2893"></td>
        <td id="LC2893" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2894" class="blob-num js-line-number" data-line-number="2894"></td>
        <td id="LC2894" class="blob-code blob-code-inner js-file-line">						<span class="pl-c">// remark: when - TimeOnNextLink &lt; 0, it means there are few seconds of left over on current link, which should be spent on next link</span></td>
      </tr>
      <tr>
        <td id="L2895" class="blob-num js-line-number" data-line-number="2895"></td>
        <td id="LC2895" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2896" class="blob-num js-line-number" data-line-number="2896"></td>
        <td id="LC2896" class="blob-code blob-code-inner js-file-line">						<span class="pl-c">///</span></td>
      </tr>
      <tr>
        <td id="L2897" class="blob-num js-line-number" data-line-number="2897"></td>
        <td id="LC2897" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2898" class="blob-num js-line-number" data-line-number="2898"></td>
        <td id="LC2898" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">if</span> (debug_flag &amp;&amp; p_Nextlink-&gt;m_FromNodeNumber == <span class="pl-c1">34</span> &amp;&amp; p_Nextlink-&gt;m_ToNodeNumber == <span class="pl-c1">30</span> &amp;&amp; CurrentTime &gt;= <span class="pl-c1">860</span>)</td>
      </tr>
      <tr>
        <td id="L2899" class="blob-num js-line-number" data-line-number="2899"></td>
        <td id="LC2899" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L2900" class="blob-num js-line-number" data-line-number="2900"></td>
        <td id="LC2900" class="blob-code blob-code-inner js-file-line">							<span class="pl-c1">TRACE</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Step 4: Time <span class="pl-c1">%f</span>, Link: <span class="pl-c1">%d</span> -&gt; <span class="pl-c1">%d</span>: vi <span class="pl-c1">%d</span>, exit time: <span class="pl-c1">%f</span>, FFTT = <span class="pl-c1">%f</span><span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>,</td>
      </tr>
      <tr>
        <td id="L2901" class="blob-num js-line-number" data-line-number="2901"></td>
        <td id="LC2901" class="blob-code blob-code-inner js-file-line">								CurrentTime, p_Nextlink-&gt;m_FromNodeNumber, p_Nextlink-&gt;m_ToNodeNumber,</td>
      </tr>
      <tr>
        <td id="L2902" class="blob-num js-line-number" data-line-number="2902"></td>
        <td id="LC2902" class="blob-code blob-code-inner js-file-line">								vi.<span class="pl-smi">veh_id</span>, vi.<span class="pl-smi">event_time_stamp</span>, FFTT);</td>
      </tr>
      <tr>
        <td id="L2903" class="blob-num js-line-number" data-line-number="2903"></td>
        <td id="LC2903" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L2904" class="blob-num js-line-number" data-line-number="2904"></td>
        <td id="LC2904" class="blob-code blob-code-inner js-file-line">						<span class="pl-c1">ASSERT</span>(vi.<span class="pl-smi">veh_id</span> &gt;= <span class="pl-c1">0</span>);</td>
      </tr>
      <tr>
        <td id="L2905" class="blob-num js-line-number" data-line-number="2905"></td>
        <td id="LC2905" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2906" class="blob-num js-line-number" data-line-number="2906"></td>
        <td id="LC2906" class="blob-code blob-code-inner js-file-line">						p_Nextlink-&gt;EntranceQueue.<span class="pl-c1">push_back</span>(vi);  <span class="pl-c">// move vehicle from current link to the entrance queue of the next link</span></td>
      </tr>
      <tr>
        <td id="L2907" class="blob-num js-line-number" data-line-number="2907"></td>
        <td id="LC2907" class="blob-code blob-code-inner js-file-line">						p_Nextlink-&gt;CFlowArrivalCount += <span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L2908" class="blob-num js-line-number" data-line-number="2908"></td>
        <td id="LC2908" class="blob-code blob-code-inner js-file-line">						pLink-&gt;CFlowDepartureCount += <span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L2909" class="blob-num js-line-number" data-line-number="2909"></td>
        <td id="LC2909" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2910" class="blob-num js-line-number" data-line-number="2910"></td>
        <td id="LC2910" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">int</span> demand_type = g_VehicleMap[vi.<span class="pl-smi">veh_id</span>]-&gt;m_DemandType;</td>
      </tr>
      <tr>
        <td id="L2911" class="blob-num js-line-number" data-line-number="2911"></td>
        <td id="LC2911" class="blob-code blob-code-inner js-file-line">						p_Nextlink-&gt;CFlowArrivalCount_DemandType[demand_type] += <span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L2912" class="blob-num js-line-number" data-line-number="2912"></td>
        <td id="LC2912" class="blob-code blob-code-inner js-file-line">						p_Nextlink-&gt;CFlowArrivalRevenue_DemandType[demand_type] += p_Nextlink-&gt;<span class="pl-c1">GetTollRateInDollar</span>(DayNo, CurrentTime, demand_type);</td>
      </tr>
      <tr>
        <td id="L2913" class="blob-num js-line-number" data-line-number="2913"></td>
        <td id="LC2913" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2914" class="blob-num js-line-number" data-line-number="2914"></td>
        <td id="LC2914" class="blob-code blob-code-inner js-file-line">						DTAVehicle* pVehicle = g_VehicleMap[vi.<span class="pl-smi">veh_id</span>];</td>
      </tr>
      <tr>
        <td id="L2915" class="blob-num js-line-number" data-line-number="2915"></td>
        <td id="LC2915" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2916" class="blob-num js-line-number" data-line-number="2916"></td>
        <td id="LC2916" class="blob-code blob-code-inner js-file-line">						pVehicle-&gt;m_TollDollarCost += p_Nextlink-&gt;<span class="pl-c1">GetTollRateInDollar</span>(DayNo, CurrentTime, demand_type);</td>
      </tr>
      <tr>
        <td id="L2917" class="blob-num js-line-number" data-line-number="2917"></td>
        <td id="LC2917" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2918" class="blob-num js-line-number" data-line-number="2918"></td>
        <td id="LC2918" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2919" class="blob-num js-line-number" data-line-number="2919"></td>
        <td id="LC2919" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">if</span> (p_Nextlink-&gt;CFlowArrivalCount != (p_Nextlink-&gt;CFlowArrivalCount_DemandType[<span class="pl-c1">1</span>] + p_Nextlink-&gt;CFlowArrivalCount_DemandType[<span class="pl-c1">2</span>] + p_Nextlink-&gt;CFlowArrivalCount_DemandType[<span class="pl-c1">3</span>] + p_Nextlink-&gt;CFlowArrivalCount_DemandType[<span class="pl-c1">4</span>]))</td>
      </tr>
      <tr>
        <td id="L2920" class="blob-num js-line-number" data-line-number="2920"></td>
        <td id="LC2920" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L2921" class="blob-num js-line-number" data-line-number="2921"></td>
        <td id="LC2921" class="blob-code blob-code-inner js-file-line">							<span class="pl-c">//						TRACE(&quot;error!&quot;);</span></td>
      </tr>
      <tr>
        <td id="L2922" class="blob-num js-line-number" data-line-number="2922"></td>
        <td id="LC2922" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L2923" class="blob-num js-line-number" data-line-number="2923"></td>
        <td id="LC2923" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2924" class="blob-num js-line-number" data-line-number="2924"></td>
        <td id="LC2924" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">if</span> (t_link_arrival_time &lt; pLink-&gt;m_LinkMOEAry.<span class="pl-c1">size</span>())</td>
      </tr>
      <tr>
        <td id="L2925" class="blob-num js-line-number" data-line-number="2925"></td>
        <td id="LC2925" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L2926" class="blob-num js-line-number" data-line-number="2926"></td>
        <td id="LC2926" class="blob-code blob-code-inner js-file-line">							pLink-&gt;m_LinkMOEAry[t_link_arrival_time].<span class="pl-smi">TotalTravelTime</span> += TravelTime;</td>
      </tr>
      <tr>
        <td id="L2927" class="blob-num js-line-number" data-line-number="2927"></td>
        <td id="LC2927" class="blob-code blob-code-inner js-file-line">							pLink-&gt;m_LinkMOEAry[t_link_arrival_time].<span class="pl-smi">TotalFlowCount</span> += <span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L2928" class="blob-num js-line-number" data-line-number="2928"></td>
        <td id="LC2928" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L2929" class="blob-num js-line-number" data-line-number="2929"></td>
        <td id="LC2929" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2930" class="blob-num js-line-number" data-line-number="2930"></td>
        <td id="LC2930" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2931" class="blob-num js-line-number" data-line-number="2931"></td>
        <td id="LC2931" class="blob-code blob-code-inner js-file-line">						pLink-&gt;departure_count += <span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L2932" class="blob-num js-line-number" data-line-number="2932"></td>
        <td id="LC2932" class="blob-code blob-code-inner js-file-line">						pLink-&gt;total_departure_based_travel_time += TravelTime;</td>
      </tr>
      <tr>
        <td id="L2933" class="blob-num js-line-number" data-line-number="2933"></td>
        <td id="LC2933" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2934" class="blob-num js-line-number" data-line-number="2934"></td>
        <td id="LC2934" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">if</span> (debug_flag &amp;&amp; vi.<span class="pl-smi">veh_id</span> == vehicle_id_trace)</td>
      </tr>
      <tr>
        <td id="L2935" class="blob-num js-line-number" data-line-number="2935"></td>
        <td id="LC2935" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L2936" class="blob-num js-line-number" data-line-number="2936"></td>
        <td id="LC2936" class="blob-code blob-code-inner js-file-line">							<span class="pl-c1">TRACE</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Step 4: target vehicle: link arrival time <span class="pl-c1">%d</span>, total travel time on current link <span class="pl-c1">%d</span>-&gt;<span class="pl-c1">%d</span>, <span class="pl-c1">%f</span><span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>, t_link_arrival_time, g_NodeVector[pLink-&gt;m_FromNodeID].<span class="pl-smi">m_NodeNumber</span>, g_NodeVector[pLink-&gt;m_ToNodeID].<span class="pl-smi">m_NodeNumber</span>, pLink-&gt;m_LinkMOEAry[t_link_arrival_time].<span class="pl-smi">TotalTravelTime</span>);</td>
      </tr>
      <tr>
        <td id="L2937" class="blob-num js-line-number" data-line-number="2937"></td>
        <td id="LC2937" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L2938" class="blob-num js-line-number" data-line-number="2938"></td>
        <td id="LC2938" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2939" class="blob-num js-line-number" data-line-number="2939"></td>
        <td id="LC2939" class="blob-code blob-code-inner js-file-line">						<span class="pl-c">//										TRACE(&quot;time %d, total travel time %f\n&quot;,t_link_arrival_time, pLink-&gt;m_LinkMOEAry[t_link_arrival_time].TotalTravelTime);</span></td>
      </tr>
      <tr>
        <td id="L2940" class="blob-num js-line-number" data-line-number="2940"></td>
        <td id="LC2940" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2941" class="blob-num js-line-number" data-line-number="2941"></td>
        <td id="LC2941" class="blob-code blob-code-inner js-file-line">						p_Nextlink-&gt;LinkInCapacity -= <span class="pl-c1">1</span>; <span class="pl-c">// reduce available space capacity by 1</span></td>
      </tr>
      <tr>
        <td id="L2942" class="blob-num js-line-number" data-line-number="2942"></td>
        <td id="LC2942" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2943" class="blob-num js-line-number" data-line-number="2943"></td>
        <td id="LC2943" class="blob-code blob-code-inner js-file-line">						<span class="pl-c">//remove this vehicle as it has moved to the final destination</span></td>
      </tr>
      <tr>
        <td id="L2944" class="blob-num js-line-number" data-line-number="2944"></td>
        <td id="LC2944" class="blob-code blob-code-inner js-file-line">						exit_queue_it = pLink-&gt;ExitQueue.<span class="pl-c1">erase</span>(exit_queue_it);</td>
      </tr>
      <tr>
        <td id="L2945" class="blob-num js-line-number" data-line-number="2945"></td>
        <td id="LC2945" class="blob-code blob-code-inner js-file-line">						vehicle_out_count--;</td>
      </tr>
      <tr>
        <td id="L2946" class="blob-num js-line-number" data-line-number="2946"></td>
        <td id="LC2946" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">continue</span>; <span class="pl-c">// it will not call &quot;++ exit_queue_it&quot; again (in the end of this while loop)</span></td>
      </tr>
      <tr>
        <td id="L2947" class="blob-num js-line-number" data-line-number="2947"></td>
        <td id="LC2947" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2948" class="blob-num js-line-number" data-line-number="2948"></td>
        <td id="LC2948" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2949" class="blob-num js-line-number" data-line-number="2949"></td>
        <td id="LC2949" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2950" class="blob-num js-line-number" data-line-number="2950"></td>
        <td id="LC2950" class="blob-code blob-code-inner js-file-line">					}</td>
      </tr>
      <tr>
        <td id="L2951" class="blob-num js-line-number" data-line-number="2951"></td>
        <td id="LC2951" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L2952" class="blob-num js-line-number" data-line-number="2952"></td>
        <td id="LC2952" class="blob-code blob-code-inner js-file-line">					{</td>
      </tr>
      <tr>
        <td id="L2953" class="blob-num js-line-number" data-line-number="2953"></td>
        <td id="LC2953" class="blob-code blob-code-inner js-file-line">						<span class="pl-c">// no capcity, do nothing, and stay in the vertical exit queue</span></td>
      </tr>
      <tr>
        <td id="L2954" class="blob-num js-line-number" data-line-number="2954"></td>
        <td id="LC2954" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2955" class="blob-num js-line-number" data-line-number="2955"></td>
        <td id="LC2955" class="blob-code blob-code-inner js-file-line">						<span class="pl-k">if</span> (TrafficFlowModelFlag == <span class="pl-c1">4</span>)  <span class="pl-c">// spatial queue with shock wave and FIFO principle: FIFO, then there is no capacity for one movement, the following vehicles cannot move even there are capacity</span></td>
      </tr>
      <tr>
        <td id="L2956" class="blob-num js-line-number" data-line-number="2956"></td>
        <td id="LC2956" class="blob-code blob-code-inner js-file-line">						{</td>
      </tr>
      <tr>
        <td id="L2957" class="blob-num js-line-number" data-line-number="2957"></td>
        <td id="LC2957" class="blob-code blob-code-inner js-file-line">							<span class="pl-k">break</span>;</td>
      </tr>
      <tr>
        <td id="L2958" class="blob-num js-line-number" data-line-number="2958"></td>
        <td id="LC2958" class="blob-code blob-code-inner js-file-line">						}</td>
      </tr>
      <tr>
        <td id="L2959" class="blob-num js-line-number" data-line-number="2959"></td>
        <td id="LC2959" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2960" class="blob-num js-line-number" data-line-number="2960"></td>
        <td id="LC2960" class="blob-code blob-code-inner js-file-line">					}</td>
      </tr>
      <tr>
        <td id="L2961" class="blob-num js-line-number" data-line-number="2961"></td>
        <td id="LC2961" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2962" class="blob-num js-line-number" data-line-number="2962"></td>
        <td id="LC2962" class="blob-code blob-code-inner js-file-line">					<span class="pl-c">//			TRACE(&quot;move veh %d from link %d link %d %d\n&quot;,vehicle_id,pLink-&gt;m_LinkNo, p_Nextlink-&gt;m_LinkNo);</span></td>
      </tr>
      <tr>
        <td id="L2963" class="blob-num js-line-number" data-line-number="2963"></td>
        <td id="LC2963" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2964" class="blob-num js-line-number" data-line-number="2964"></td>
        <td id="LC2964" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2965" class="blob-num js-line-number" data-line-number="2965"></td>
        <td id="LC2965" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L2966" class="blob-num js-line-number" data-line-number="2966"></td>
        <td id="LC2966" class="blob-code blob-code-inner js-file-line">				<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L2967" class="blob-num js-line-number" data-line-number="2967"></td>
        <td id="LC2967" class="blob-code blob-code-inner js-file-line">				{</td>
      </tr>
      <tr>
        <td id="L2968" class="blob-num js-line-number" data-line-number="2968"></td>
        <td id="LC2968" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2969" class="blob-num js-line-number" data-line-number="2969"></td>
        <td id="LC2969" class="blob-code blob-code-inner js-file-line">					<span class="pl-c">// reach destination, increase the counter.</span></td>
      </tr>
      <tr>
        <td id="L2970" class="blob-num js-line-number" data-line-number="2970"></td>
        <td id="LC2970" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">float</span> ArrivalTimeOnDSN = vi.<span class="pl-smi">event_time_stamp</span>;  <span class="pl-c">// no delay at destination node</span></td>
      </tr>
      <tr>
        <td id="L2971" class="blob-num js-line-number" data-line-number="2971"></td>
        <td id="LC2971" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2972" class="blob-num js-line-number" data-line-number="2972"></td>
        <td id="LC2972" class="blob-code blob-code-inner js-file-line">					<span class="pl-c">// update statistics for traveled link</span></td>
      </tr>
      <tr>
        <td id="L2973" class="blob-num js-line-number" data-line-number="2973"></td>
        <td id="LC2973" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">int</span> link_sequence_no = g_VehicleMap[vehicle_id]-&gt;m_SimLinkSequenceNo;</td>
      </tr>
      <tr>
        <td id="L2974" class="blob-num js-line-number" data-line-number="2974"></td>
        <td id="LC2974" class="blob-code blob-code-inner js-file-line">					g_VehicleMap[vehicle_id]-&gt;m_LinkAry[link_sequence_no].<span class="pl-smi">AbsArrivalTimeOnDSN</span> = ArrivalTimeOnDSN;</td>
      </tr>
      <tr>
        <td id="L2975" class="blob-num js-line-number" data-line-number="2975"></td>
        <td id="LC2975" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">float</span> TravelTime = <span class="pl-c1">0</span>;</td>
      </tr>
      <tr>
        <td id="L2976" class="blob-num js-line-number" data-line-number="2976"></td>
        <td id="LC2976" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2977" class="blob-num js-line-number" data-line-number="2977"></td>
        <td id="LC2977" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">if</span> (link_sequence_no &gt;= <span class="pl-c1">1</span>)</td>
      </tr>
      <tr>
        <td id="L2978" class="blob-num js-line-number" data-line-number="2978"></td>
        <td id="LC2978" class="blob-code blob-code-inner js-file-line">					{</td>
      </tr>
      <tr>
        <td id="L2979" class="blob-num js-line-number" data-line-number="2979"></td>
        <td id="LC2979" class="blob-code blob-code-inner js-file-line">						TravelTime = g_VehicleMap[vehicle_id]-&gt;m_LinkAry[link_sequence_no].<span class="pl-smi">AbsArrivalTimeOnDSN</span> -</td>
      </tr>
      <tr>
        <td id="L2980" class="blob-num js-line-number" data-line-number="2980"></td>
        <td id="LC2980" class="blob-code blob-code-inner js-file-line">							g_VehicleMap[vehicle_id]-&gt;m_LinkAry[link_sequence_no - <span class="pl-c1">1</span>].<span class="pl-smi">AbsArrivalTimeOnDSN</span>;</td>
      </tr>
      <tr>
        <td id="L2981" class="blob-num js-line-number" data-line-number="2981"></td>
        <td id="LC2981" class="blob-code blob-code-inner js-file-line">					}</td>
      </tr>
      <tr>
        <td id="L2982" class="blob-num js-line-number" data-line-number="2982"></td>
        <td id="LC2982" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">else</span></td>
      </tr>
      <tr>
        <td id="L2983" class="blob-num js-line-number" data-line-number="2983"></td>
        <td id="LC2983" class="blob-code blob-code-inner js-file-line">					{</td>
      </tr>
      <tr>
        <td id="L2984" class="blob-num js-line-number" data-line-number="2984"></td>
        <td id="LC2984" class="blob-code blob-code-inner js-file-line">						TravelTime = g_VehicleMap[vehicle_id]-&gt;m_LinkAry[link_sequence_no].<span class="pl-smi">AbsArrivalTimeOnDSN</span> -</td>
      </tr>
      <tr>
        <td id="L2985" class="blob-num js-line-number" data-line-number="2985"></td>
        <td id="LC2985" class="blob-code blob-code-inner js-file-line">							g_VehicleMap[vehicle_id]-&gt;m_DepartureTime;</td>
      </tr>
      <tr>
        <td id="L2986" class="blob-num js-line-number" data-line-number="2986"></td>
        <td id="LC2986" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2987" class="blob-num js-line-number" data-line-number="2987"></td>
        <td id="LC2987" class="blob-code blob-code-inner js-file-line">					}</td>
      </tr>
      <tr>
        <td id="L2988" class="blob-num js-line-number" data-line-number="2988"></td>
        <td id="LC2988" class="blob-code blob-code-inner js-file-line">					g_VehicleMap[vehicle_id]-&gt;m_Delay += (TravelTime - pLink-&gt;m_FreeFlowTravelTime);</td>
      </tr>
      <tr>
        <td id="L2989" class="blob-num js-line-number" data-line-number="2989"></td>
        <td id="LC2989" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2990" class="blob-num js-line-number" data-line-number="2990"></td>
        <td id="LC2990" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2991" class="blob-num js-line-number" data-line-number="2991"></td>
        <td id="LC2991" class="blob-code blob-code-inner js-file-line">					<span class="pl-c">// finally move to next link</span></td>
      </tr>
      <tr>
        <td id="L2992" class="blob-num js-line-number" data-line-number="2992"></td>
        <td id="LC2992" class="blob-code blob-code-inner js-file-line">					g_VehicleMap[vehicle_id]-&gt;m_SimLinkSequenceNo = g_VehicleMap[vehicle_id]-&gt;m_SimLinkSequenceNo + <span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L2993" class="blob-num js-line-number" data-line-number="2993"></td>
        <td id="LC2993" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2994" class="blob-num js-line-number" data-line-number="2994"></td>
        <td id="LC2994" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2995" class="blob-num js-line-number" data-line-number="2995"></td>
        <td id="LC2995" class="blob-code blob-code-inner js-file-line">					g_VehicleMap[vehicle_id]-&gt;m_ArrivalTime = ArrivalTimeOnDSN;</td>
      </tr>
      <tr>
        <td id="L2996" class="blob-num js-line-number" data-line-number="2996"></td>
        <td id="LC2996" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2997" class="blob-num js-line-number" data-line-number="2997"></td>
        <td id="LC2997" class="blob-code blob-code-inner js-file-line">					g_VehicleMap[vehicle_id]-&gt;m_TripTime = g_VehicleMap[vehicle_id]-&gt;m_ArrivalTime - g_VehicleMap[vehicle_id]-&gt;m_DepartureTime;</td>
      </tr>
      <tr>
        <td id="L2998" class="blob-num js-line-number" data-line-number="2998"></td>
        <td id="LC2998" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L2999" class="blob-num js-line-number" data-line-number="2999"></td>
        <td id="LC2999" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">if</span> (debug_flag &amp;&amp; vi.<span class="pl-smi">veh_id</span> == vehicle_id_trace)</td>
      </tr>
      <tr>
        <td id="L3000" class="blob-num js-line-number" data-line-number="3000"></td>
        <td id="LC3000" class="blob-code blob-code-inner js-file-line">					{</td>
      </tr>
      <tr>
        <td id="L3001" class="blob-num js-line-number" data-line-number="3001"></td>
        <td id="LC3001" class="blob-code blob-code-inner js-file-line">						<span class="pl-c1">TRACE</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>Step 4: time <span class="pl-c1">%f</span>, target vehicle reaches the destination, vehicle trip time: <span class="pl-c1">%f</span>, # of links  = <span class="pl-c1">%d</span><span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>,</td>
      </tr>
      <tr>
        <td id="L3002" class="blob-num js-line-number" data-line-number="3002"></td>
        <td id="LC3002" class="blob-code blob-code-inner js-file-line">							ArrivalTimeOnDSN, g_VehicleMap[vehicle_id]-&gt;m_TripTime,</td>
      </tr>
      <tr>
        <td id="L3003" class="blob-num js-line-number" data-line-number="3003"></td>
        <td id="LC3003" class="blob-code blob-code-inner js-file-line">							g_VehicleMap[vehicle_id]-&gt;m_SimLinkSequenceNo);</td>
      </tr>
      <tr>
        <td id="L3004" class="blob-num js-line-number" data-line-number="3004"></td>
        <td id="LC3004" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3005" class="blob-num js-line-number" data-line-number="3005"></td>
        <td id="LC3005" class="blob-code blob-code-inner js-file-line">						vehicle_id_trace = -<span class="pl-c1">1</span>; <span class="pl-c">// not tracking anymore </span></td>
      </tr>
      <tr>
        <td id="L3006" class="blob-num js-line-number" data-line-number="3006"></td>
        <td id="LC3006" class="blob-code blob-code-inner js-file-line">					}</td>
      </tr>
      <tr>
        <td id="L3007" class="blob-num js-line-number" data-line-number="3007"></td>
        <td id="LC3007" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3008" class="blob-num js-line-number" data-line-number="3008"></td>
        <td id="LC3008" class="blob-code blob-code-inner js-file-line">					g_VehicleMap[vehicle_id]-&gt;m_bComplete = <span class="pl-c1">true</span>;</td>
      </tr>
      <tr>
        <td id="L3009" class="blob-num js-line-number" data-line-number="3009"></td>
        <td id="LC3009" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">int</span> OriginDepartureTime = (<span class="pl-k">int</span>)(g_VehicleMap[vehicle_id]-&gt;m_DepartureTime);</td>
      </tr>
      <tr>
        <td id="L3010" class="blob-num js-line-number" data-line-number="3010"></td>
        <td id="LC3010" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3011" class="blob-num js-line-number" data-line-number="3011"></td>
        <td id="LC3011" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3012" class="blob-num js-line-number" data-line-number="3012"></td>
        <td id="LC3012" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">pragma</span> omp critical  <span class="pl-c">// keep this section as a single thread as it involves network-wide statistics collection</span></td>
      </tr>
      <tr>
        <td id="L3013" class="blob-num js-line-number" data-line-number="3013"></td>
        <td id="LC3013" class="blob-code blob-code-inner js-file-line">					{</td>
      </tr>
      <tr>
        <td id="L3014" class="blob-num js-line-number" data-line-number="3014"></td>
        <td id="LC3014" class="blob-code blob-code-inner js-file-line">						g_NetworkMOEAry[OriginDepartureTime].<span class="pl-smi">AbsArrivalTimeOnDSN_in_a_min</span> += g_VehicleMap[vehicle_id]-&gt;m_TripTime;</td>
      </tr>
      <tr>
        <td id="L3015" class="blob-num js-line-number" data-line-number="3015"></td>
        <td id="LC3015" class="blob-code blob-code-inner js-file-line">						g_Number_of_CompletedVehicles += <span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L3016" class="blob-num js-line-number" data-line-number="3016"></td>
        <td id="LC3016" class="blob-code blob-code-inner js-file-line">						g_NetworkMOEAry[time_stamp_in_min].<span class="pl-smi">CumulativeOutFlow</span> = g_Number_of_CompletedVehicles;</td>
      </tr>
      <tr>
        <td id="L3017" class="blob-num js-line-number" data-line-number="3017"></td>
        <td id="LC3017" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3018" class="blob-num js-line-number" data-line-number="3018"></td>
        <td id="LC3018" class="blob-code blob-code-inner js-file-line">					}</td>
      </tr>
      <tr>
        <td id="L3019" class="blob-num js-line-number" data-line-number="3019"></td>
        <td id="LC3019" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3020" class="blob-num js-line-number" data-line-number="3020"></td>
        <td id="LC3020" class="blob-code blob-code-inner js-file-line">					pLink-&gt;CFlowDepartureCount += <span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L3021" class="blob-num js-line-number" data-line-number="3021"></td>
        <td id="LC3021" class="blob-code blob-code-inner js-file-line">					pLink-&gt;m_LinkMOEAry[t_link_arrival_time].<span class="pl-smi">TotalTravelTime</span> += TravelTime;</td>
      </tr>
      <tr>
        <td id="L3022" class="blob-num js-line-number" data-line-number="3022"></td>
        <td id="LC3022" class="blob-code blob-code-inner js-file-line">					pLink-&gt;m_LinkMOEAry[t_link_arrival_time].<span class="pl-smi">TotalFlowCount</span> += <span class="pl-c1">1</span>;</td>
      </tr>
      <tr>
        <td id="L3023" class="blob-num js-line-number" data-line-number="3023"></td>
        <td id="LC3023" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3024" class="blob-num js-line-number" data-line-number="3024"></td>
        <td id="LC3024" class="blob-code blob-code-inner js-file-line">					<span class="pl-c">//remove this vehicle as it has moved to the next link</span></td>
      </tr>
      <tr>
        <td id="L3025" class="blob-num js-line-number" data-line-number="3025"></td>
        <td id="LC3025" class="blob-code blob-code-inner js-file-line">					exit_queue_it = pLink-&gt;ExitQueue.<span class="pl-c1">erase</span>(exit_queue_it);</td>
      </tr>
      <tr>
        <td id="L3026" class="blob-num js-line-number" data-line-number="3026"></td>
        <td id="LC3026" class="blob-code blob-code-inner js-file-line">					vehicle_out_count--;</td>
      </tr>
      <tr>
        <td id="L3027" class="blob-num js-line-number" data-line-number="3027"></td>
        <td id="LC3027" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3028" class="blob-num js-line-number" data-line-number="3028"></td>
        <td id="LC3028" class="blob-code blob-code-inner js-file-line">					<span class="pl-k">continue</span>; <span class="pl-c">// it will not call &quot;++ exit_queue_it&quot; again (in the end of this while loop)</span></td>
      </tr>
      <tr>
        <td id="L3029" class="blob-num js-line-number" data-line-number="3029"></td>
        <td id="LC3029" class="blob-code blob-code-inner js-file-line">				}</td>
      </tr>
      <tr>
        <td id="L3030" class="blob-num js-line-number" data-line-number="3030"></td>
        <td id="LC3030" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3031" class="blob-num js-line-number" data-line-number="3031"></td>
        <td id="LC3031" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3032" class="blob-num js-line-number" data-line-number="3032"></td>
        <td id="LC3032" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3033" class="blob-num js-line-number" data-line-number="3033"></td>
        <td id="LC3033" class="blob-code blob-code-inner js-file-line">				++exit_queue_it;</td>
      </tr>
      <tr>
        <td id="L3034" class="blob-num js-line-number" data-line-number="3034"></td>
        <td id="LC3034" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L3035" class="blob-num js-line-number" data-line-number="3035"></td>
        <td id="LC3035" class="blob-code blob-code-inner js-file-line">			incoming_link_count++;</td>
      </tr>
      <tr>
        <td id="L3036" class="blob-num js-line-number" data-line-number="3036"></td>
        <td id="LC3036" class="blob-code blob-code-inner js-file-line">			incoming_link_sequence = (incoming_link_sequence + <span class="pl-c1">1</span>) % IncomingLinkSize;  <span class="pl-c">// increase incoming_link_sequence by 1 within IncomingLinkSize</span></td>
      </tr>
      <tr>
        <td id="L3037" class="blob-num js-line-number" data-line-number="3037"></td>
        <td id="LC3037" class="blob-code blob-code-inner js-file-line">		}  <span class="pl-c">// for each incoming link</span></td>
      </tr>
      <tr>
        <td id="L3038" class="blob-num js-line-number" data-line-number="3038"></td>
        <td id="LC3038" class="blob-code blob-code-inner js-file-line">	} <span class="pl-c">// for each node</span></td>
      </tr>
      <tr>
        <td id="L3039" class="blob-num js-line-number" data-line-number="3039"></td>
        <td id="LC3039" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3040" class="blob-num js-line-number" data-line-number="3040"></td>
        <td id="LC3040" class="blob-code blob-code-inner js-file-line">	<span class="pl-c">//	step 5: statistics collection</span></td>
      </tr>
      <tr>
        <td id="L3041" class="blob-num js-line-number" data-line-number="3041"></td>
        <td id="LC3041" class="blob-code blob-code-inner js-file-line">#<span class="pl-k">pragma</span> omp parallel for</td>
      </tr>
      <tr>
        <td id="L3042" class="blob-num js-line-number" data-line-number="3042"></td>
        <td id="LC3042" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">for</span> (<span class="pl-k">int</span> li = <span class="pl-c1">0</span>; li&lt; link_size; li++)</td>
      </tr>
      <tr>
        <td id="L3043" class="blob-num js-line-number" data-line-number="3043"></td>
        <td id="LC3043" class="blob-code blob-code-inner js-file-line">	{</td>
      </tr>
      <tr>
        <td id="L3044" class="blob-num js-line-number" data-line-number="3044"></td>
        <td id="LC3044" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3045" class="blob-num js-line-number" data-line-number="3045"></td>
        <td id="LC3045" class="blob-code blob-code-inner js-file-line">		<span class="pl-c">// Cumulative flow counts</span></td>
      </tr>
      <tr>
        <td id="L3046" class="blob-num js-line-number" data-line-number="3046"></td>
        <td id="LC3046" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3047" class="blob-num js-line-number" data-line-number="3047"></td>
        <td id="LC3047" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">int</span> t_residual = meso_simulation_time_interval_no % MAX_TIME_INTERVAL_ADCURVE;</td>
      </tr>
      <tr>
        <td id="L3048" class="blob-num js-line-number" data-line-number="3048"></td>
        <td id="LC3048" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3049" class="blob-num js-line-number" data-line-number="3049"></td>
        <td id="LC3049" class="blob-code blob-code-inner js-file-line">		DTALink* pLink = g_LinkVector[li];</td>
      </tr>
      <tr>
        <td id="L3050" class="blob-num js-line-number" data-line-number="3050"></td>
        <td id="LC3050" class="blob-code blob-code-inner js-file-line">		pLink-&gt;m_CumuArrivalFlow[t_residual] = pLink-&gt;CFlowArrivalCount;</td>
      </tr>
      <tr>
        <td id="L3051" class="blob-num js-line-number" data-line-number="3051"></td>
        <td id="LC3051" class="blob-code blob-code-inner js-file-line">		pLink-&gt;m_CumuDeparturelFlow[t_residual] = pLink-&gt;CFlowDepartureCount;</td>
      </tr>
      <tr>
        <td id="L3052" class="blob-num js-line-number" data-line-number="3052"></td>
        <td id="LC3052" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3053" class="blob-num js-line-number" data-line-number="3053"></td>
        <td id="LC3053" class="blob-code blob-code-inner js-file-line">		<span class="pl-k">if</span> (meso_simulation_time_interval_no%g_number_of_intervals_per_min == <span class="pl-c1">0</span>)  <span class="pl-c">// per min statistics</span></td>
      </tr>
      <tr>
        <td id="L3054" class="blob-num js-line-number" data-line-number="3054"></td>
        <td id="LC3054" class="blob-code blob-code-inner js-file-line">		{</td>
      </tr>
      <tr>
        <td id="L3055" class="blob-num js-line-number" data-line-number="3055"></td>
        <td id="LC3055" class="blob-code blob-code-inner js-file-line">			pLink-&gt;VehicleCount = pLink-&gt;CFlowArrivalCount - pLink-&gt;CFlowDepartureCount;</td>
      </tr>
      <tr>
        <td id="L3056" class="blob-num js-line-number" data-line-number="3056"></td>
        <td id="LC3056" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3057" class="blob-num js-line-number" data-line-number="3057"></td>
        <td id="LC3057" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">// queue is the number of vehicles at the end of simulation interval</span></td>
      </tr>
      <tr>
        <td id="L3058" class="blob-num js-line-number" data-line-number="3058"></td>
        <td id="LC3058" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3059" class="blob-num js-line-number" data-line-number="3059"></td>
        <td id="LC3059" class="blob-code blob-code-inner js-file-line">			pLink-&gt;m_LinkMOEAry[time_stamp_in_min].<span class="pl-smi">ExitQueueLength</span> = pLink-&gt;ExitQueue.<span class="pl-c1">size</span>();</td>
      </tr>
      <tr>
        <td id="L3060" class="blob-num js-line-number" data-line-number="3060"></td>
        <td id="LC3060" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3061" class="blob-num js-line-number" data-line-number="3061"></td>
        <td id="LC3061" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (pLink-&gt;m_LinkMOEAry[time_stamp_in_min].<span class="pl-smi">ExitQueueLength</span> &gt;= <span class="pl-c1">1</span> &amp;&amp; pLink-&gt;m_LinkMOEAry[time_stamp_in_min].<span class="pl-smi">TrafficStateCode</span> != <span class="pl-c1">2</span>)   <span class="pl-c">// not fully congested</span></td>
      </tr>
      <tr>
        <td id="L3062" class="blob-num js-line-number" data-line-number="3062"></td>
        <td id="LC3062" class="blob-code blob-code-inner js-file-line">				pLink-&gt;m_LinkMOEAry[time_stamp_in_min].<span class="pl-smi">TrafficStateCode</span> = <span class="pl-c1">1</span>;  <span class="pl-c">// partially congested</span></td>
      </tr>
      <tr>
        <td id="L3063" class="blob-num js-line-number" data-line-number="3063"></td>
        <td id="LC3063" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3064" class="blob-num js-line-number" data-line-number="3064"></td>
        <td id="LC3064" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">// time_stamp_in_min+1 is because we take the stastistics to next time stamp</span></td>
      </tr>
      <tr>
        <td id="L3065" class="blob-num js-line-number" data-line-number="3065"></td>
        <td id="LC3065" class="blob-code blob-code-inner js-file-line">			pLink-&gt;m_LinkMOEAry[time_stamp_in_min].<span class="pl-smi">CumulativeArrivalCount</span> = pLink-&gt;CFlowArrivalCount;</td>
      </tr>
      <tr>
        <td id="L3066" class="blob-num js-line-number" data-line-number="3066"></td>
        <td id="LC3066" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3067" class="blob-num js-line-number" data-line-number="3067"></td>
        <td id="LC3067" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">// toll collection </span></td>
      </tr>
      <tr>
        <td id="L3068" class="blob-num js-line-number" data-line-number="3068"></td>
        <td id="LC3068" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3069" class="blob-num js-line-number" data-line-number="3069"></td>
        <td id="LC3069" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">//for(int pt = 1; pt &lt; MAX_DEMAND_TYPE_SIZE; pt++)</span></td>
      </tr>
      <tr>
        <td id="L3070" class="blob-num js-line-number" data-line-number="3070"></td>
        <td id="LC3070" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">//{</span></td>
      </tr>
      <tr>
        <td id="L3071" class="blob-num js-line-number" data-line-number="3071"></td>
        <td id="LC3071" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">//	pLink-&gt;m_LinkMOEAry [time_stamp_in_min].CumulativeArrivalCount_DemandType[pt] = pLink-&gt;CFlowArrivalCount_DemandType[pt];</span></td>
      </tr>
      <tr>
        <td id="L3072" class="blob-num js-line-number" data-line-number="3072"></td>
        <td id="LC3072" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">//	pLink-&gt;m_LinkMOEAry [time_stamp_in_min].CumulativeRevenue_DemandType[pt] = pLink-&gt;CFlowArrivalRevenue_DemandType[pt];</span></td>
      </tr>
      <tr>
        <td id="L3073" class="blob-num js-line-number" data-line-number="3073"></td>
        <td id="LC3073" class="blob-code blob-code-inner js-file-line">			<span class="pl-c">//}</span></td>
      </tr>
      <tr>
        <td id="L3074" class="blob-num js-line-number" data-line-number="3074"></td>
        <td id="LC3074" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3075" class="blob-num js-line-number" data-line-number="3075"></td>
        <td id="LC3075" class="blob-code blob-code-inner js-file-line">			pLink-&gt;m_LinkMOEAry[time_stamp_in_min].<span class="pl-smi">CumulativeDepartureCount</span> = pLink-&gt;CFlowDepartureCount;</td>
      </tr>
      <tr>
        <td id="L3076" class="blob-num js-line-number" data-line-number="3076"></td>
        <td id="LC3076" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3077" class="blob-num js-line-number" data-line-number="3077"></td>
        <td id="LC3077" class="blob-code blob-code-inner js-file-line">			<span class="pl-k">if</span> (debug_flag &amp;&amp; link_id_trace == li &amp;&amp; vehicle_id_trace &gt;= <span class="pl-c1">0</span>)</td>
      </tr>
      <tr>
        <td id="L3078" class="blob-num js-line-number" data-line-number="3078"></td>
        <td id="LC3078" class="blob-code blob-code-inner js-file-line">			{</td>
      </tr>
      <tr>
        <td id="L3079" class="blob-num js-line-number" data-line-number="3079"></td>
        <td id="LC3079" class="blob-code blob-code-inner js-file-line">				<span class="pl-c1">TRACE</span>(<span class="pl-s"><span class="pl-pds">&quot;</span>step 5: statistics collection: Time <span class="pl-c1">%d</span>, link <span class="pl-c1">%d</span> -&gt; <span class="pl-c1">%d</span>, Cumulative arrival count <span class="pl-c1">%d</span>, cumulative departure count <span class="pl-c1">%d</span> <span class="pl-cce">\n</span><span class="pl-pds">&quot;</span></span>, time_stamp_in_min, g_NodeVector[pLink-&gt;m_FromNodeID].<span class="pl-smi">m_NodeNumber</span>, g_NodeVector[pLink-&gt;m_ToNodeID].<span class="pl-smi">m_NodeNumber</span>,</td>
      </tr>
      <tr>
        <td id="L3080" class="blob-num js-line-number" data-line-number="3080"></td>
        <td id="LC3080" class="blob-code blob-code-inner js-file-line">					pLink-&gt;m_LinkMOEAry[time_stamp_in_min].<span class="pl-smi">CumulativeArrivalCount</span>, pLink-&gt;m_LinkMOEAry[time_stamp_in_min].<span class="pl-smi">CumulativeDepartureCount</span>);</td>
      </tr>
      <tr>
        <td id="L3081" class="blob-num js-line-number" data-line-number="3081"></td>
        <td id="LC3081" class="blob-code blob-code-inner js-file-line">			}</td>
      </tr>
      <tr>
        <td id="L3082" class="blob-num js-line-number" data-line-number="3082"></td>
        <td id="LC3082" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3083" class="blob-num js-line-number" data-line-number="3083"></td>
        <td id="LC3083" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3084" class="blob-num js-line-number" data-line-number="3084"></td>
        <td id="LC3084" class="blob-code blob-code-inner js-file-line">		}</td>
      </tr>
      <tr>
        <td id="L3085" class="blob-num js-line-number" data-line-number="3085"></td>
        <td id="LC3085" class="blob-code blob-code-inner js-file-line">	}</td>
      </tr>
      <tr>
        <td id="L3086" class="blob-num js-line-number" data-line-number="3086"></td>
        <td id="LC3086" class="blob-code blob-code-inner js-file-line">	<span class="pl-k">return</span> <span class="pl-c1">true</span>;</td>
      </tr>
      <tr>
        <td id="L3087" class="blob-num js-line-number" data-line-number="3087"></td>
        <td id="LC3087" class="blob-code blob-code-inner js-file-line">}</td>
      </tr>
      <tr>
        <td id="L3088" class="blob-num js-line-number" data-line-number="3088"></td>
        <td id="LC3088" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
      <tr>
        <td id="L3089" class="blob-num js-line-number" data-line-number="3089"></td>
        <td id="LC3089" class="blob-code blob-code-inner js-file-line">
</td>
      </tr>
</table>

  </div>

</div>

<a href="#jump-to-line" rel="facebox[.linejump]" data-hotkey="l" style="display:none">Jump to Line</a>
<div id="jump-to-line" style="display:none">
  <!-- </textarea> --><!-- '"` --><form accept-charset="UTF-8" action="" class="js-jump-to-line-form" method="get"><div style="margin:0;padding:0;display:inline"><input name="utf8" type="hidden" value="&#x2713;" /></div>
    <input class="linejump-input js-jump-to-line-field" type="text" placeholder="Jump to line&hellip;" aria-label="Jump to line" autofocus>
    <button type="submit" class="btn">Go</button>
</form></div>

        </div>
      </div>
      <div class="modal-backdrop"></div>
    </div>
  </div>


    </div>

      <div class="container">
  <div class="site-footer" role="contentinfo">
    <ul class="site-footer-links right">
        <li><a href="https://status.github.com/" data-ga-click="Footer, go to status, text:status">Status</a></li>
      <li><a href="https://developer.github.com" data-ga-click="Footer, go to api, text:api">API</a></li>
      <li><a href="https://training.github.com" data-ga-click="Footer, go to training, text:training">Training</a></li>
      <li><a href="https://shop.github.com" data-ga-click="Footer, go to shop, text:shop">Shop</a></li>
        <li><a href="https://github.com/blog" data-ga-click="Footer, go to blog, text:blog">Blog</a></li>
        <li><a href="https://github.com/about" data-ga-click="Footer, go to about, text:about">About</a></li>
        <li><a href="https://github.com/pricing" data-ga-click="Footer, go to pricing, text:pricing">Pricing</a></li>

    </ul>

    <a href="https://github.com" aria-label="Homepage">
      <span class="mega-octicon octicon-mark-github" title="GitHub"></span>
</a>
    <ul class="site-footer-links">
      <li>&copy; 2015 <span title="0.13213s from github-fe137-cp1-prd.iad.github.net">GitHub</span>, Inc.</li>
        <li><a href="https://github.com/site/terms" data-ga-click="Footer, go to terms, text:terms">Terms</a></li>
        <li><a href="https://github.com/site/privacy" data-ga-click="Footer, go to privacy, text:privacy">Privacy</a></li>
        <li><a href="https://github.com/security" data-ga-click="Footer, go to security, text:security">Security</a></li>
        <li><a href="https://github.com/contact" data-ga-click="Footer, go to contact, text:contact">Contact</a></li>
        <li><a href="https://help.github.com" data-ga-click="Footer, go to help, text:help">Help</a></li>
    </ul>
  </div>
</div>



    
    
    

    <div id="ajax-error-message" class="flash flash-error">
      <span class="octicon octicon-alert"></span>
      <button type="button" class="flash-close js-flash-close js-ajax-error-dismiss" aria-label="Dismiss error">
        <span class="octicon octicon-x"></span>
      </button>
      Something went wrong with that request. Please try again.
    </div>


      <script crossorigin="anonymous" integrity="sha256-+Ec97OckLaaiDVIxNjSIGzl1xSzrqh5sOBV8DyYYVpE=" src="https://assets-cdn.github.com/assets/frameworks-f8473dece7242da6a20d52313634881b3975c52cebaa1e6c38157c0f26185691.js"></script>
      <script async="async" crossorigin="anonymous" integrity="sha256-m/Opn3sG+OjWBq0zNSqiMtGtBf/m8WDZomr344srYOU=" src="https://assets-cdn.github.com/assets/github-9bf3a99f7b06f8e8d606ad33352aa232d1ad05ffe6f160d9a26af7e38b2b60e5.js"></script>
      
      
    <div class="js-stale-session-flash stale-session-flash flash flash-warn flash-banner hidden">
      <span class="octicon octicon-alert"></span>
      <span class="signed-in-tab-flash">You signed in with another tab or window. <a href="">Reload</a> to refresh your session.</span>
      <span class="signed-out-tab-flash">You signed out in another tab or window. <a href="">Reload</a> to refresh your session.</span>
    </div>
  </body>
</html>

