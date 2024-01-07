function X0q = Convert2Quat(X0)
    top = X0(1:6);
    bot = X0(10:12);
    quats = e2q(X0(7:9));
    quats = Normalise(quats);
    X0q = [top;quats;bot];
return
    