data <- read.table("metrics.txt", header = T)

data.0 <- data[data$Agent==0,]
data.1 <- data[data$Agent!=0,]

par(mfrow=c(1,2))
boxplot(data.0$distanceToNearestPerson,
        data.1$distanceToNearestPerson,
        outline=F,col=rainbow(2),names = c("Robot","Human"),xlab="Agent type",ylab = "Meters",
        main="Distance to nearest agent")

boxplot(data.0$distanceToNearestObstacle,
        data.1$distanceToNearestObstacle,
        outline=F,col=rainbow(2),names = c("Robot","Human"),xlab="Agent type",ylab = "Meters",
        main="Distance to nearest obstacle")

par(mfrow=c(1,1))
d0 <- density(data.0$Velocity)
plot(d0, main="Robot velocity PDF",xlab = "m/s")

d1 <- density(data.1$Velocity)
plot(d1, main="People velocity PDF",xlab = "m/s")

