#usage is ./autoGit.sh <commit message>
git add -A
git commit -m "$*"
git pull
git push