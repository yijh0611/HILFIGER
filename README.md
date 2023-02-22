# HILFIGER       
### Notion link
https://fluff-hole-6c7.notion.site/AiMS-style-a2ee1152c8e34b55a8d0f3f258e0da82   

### Schedule      
Project Deadline: ~ 3월 22일<br>
Regular Meeting : 월요일 오후 3시(15시)<br>     
      
### Part division       
- BenchMark-1: 준학(main), 우현(main), 동희, 수빈    
- BenchMark-2: 동희(main), 우현      
- BenchMark-3: 수빈(main), 동희, 준학
<br>
<br>

### Configuration in the workspace
git init
<br>
git config --global user.name "your_name"
<br>
git config --global user.email "your_email"
<br>
git remote add origin https://github.com/yijh0611/HILFIGER.git
<br>
<br>

### Delete config
git config --unset --global user.name "your_name"
<br>
git config --unset --global user.email "your_email"
<br>
<br>
### How to commit
git add .      # add all files
<br>
git commit -m "commit message"
<br>
<br>
### Master(main) guide
git branch             
<br>
git branch -M main
<br>
git push -u origin main  # once
<br>
git push
<br>
<br>
### Members guide
git branch {branch_name}   # create your branch
<br>
git checkout {branch_name} # change the master(main) branch to your branch
<br>
git push origin {branch_name}
<br>
<br>
### synchronization
git pull 
<br>
or
<br>
git pull origin {desired_branch_name}
<br>
<br>
### 주의
Repository에서 여러 개의 files을 변경했을 경우,
모든 파일을 add 하지 말고 변경된 이력만 add하자.
<br>
- step 1: 변경된 이력들 확인 
<br>
$ git status
<br>
그 결과로 untracked files들이 빨간색 글씨로 보인다.
<br>
- step 2-1: 첫 번째 파일 add
$ git add [파일명]
<br>
- step 3-1. 첫 번째 파일 commit
$ git commit -m "해당 파일에 대해 간략한 수정 내용"
<br>
- step 2-2: 두 번째 파일 add
<br>
- step 3-2: 두 번째 파일 commit
<br>
...
<br>
- step 4: 최종적으로 remote repository에 push
$ git push
<br>
