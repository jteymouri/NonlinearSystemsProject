function f = gaussian(x,y,Xmean,Ymean,xvar,yvar)
    f = exp(-((x-Xmean)^2)/(2*xvar^2) + ((y-Ymean)^2)/(2*yvar^2));
end