# HILFIGER
<Notion link>
https://fluff-hole-6c7.notion.site/AiMS-style-a2ee1152c8e34b55a8d0f3f258e0da82

<Schedule>
Project Deadline: ~ 3월 22일
Regular Meeting : 월요일 오후 3시(15시)


<Part division>
BenchMark-1: 준학(main), 우현(main), 동희, 수빈
BenchMark-2: 동희(main), 우현
BenchMark-3: 수빈(main), 동희, 준학



## Configuration in the workspace
git init
git config --global user.name "your_name"
git config --global user.email "your_email"
git remote add origin https://github.com/yijh0611/HILFIGER.git


## Delete config
git config --unset --global user.name "your_name"
git config --unset --global user.email "your_email"


## How to commit
git add .      # add all files
git commit -m "commit message"


## Master(main) guide
git branch             
git branch -M main
git push -u origin main  # once
git push


## Members guide
git branch {branch_name}   # create your branch
git checkout {branch_name} # change the master(main) branch to your branch
git push origin {branch_name}


## synchronization
git pull 
# or
git pull origin {desired_branch_name}


##---------------필독-----------------##
Repository에서 여러 개의 files을 변경했을 경우,
모든 파일을 add 하지 말고 변경된 이력만 add하자.

step 1: 변경된 이력들 확인 
$ git status
그 결과로
untracked files들이 빨간색 글씨로 보인다.

step 2-1: 첫 번째 파일 add
$ git add [파일명]

step 3-1. 첫 번째 파일 commit
$ git commit -m "해당 파일에 대해 간략한 수정 내용"

step 2-2: 두 번째 파일 add

step 3-2: 두 번째 파일 commit

...

step 4: 최종적으로 remote repository에 push
$ git push
